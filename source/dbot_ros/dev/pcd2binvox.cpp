//
// pcd2binvox
// Convert a .pcd file to .binvox
//
// pcd is Point Cloud Data from PCL (PointCloud Library).
// binvox is a binary format for a 3D voxel grid.
//
// David Butterworth, 2016.
//
// binvox was developed by Patrick Min (www.patrickmin.com)
// The RLE code below is based on binvox-rw-py by Daniel Maturana.
//

#include <ros/ros.h>

#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip> // setprecision
#include <stdlib.h>
#include <mutex>

#include <cstdlib>
#include <cstring>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/dynamic_bitset.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <dbot_ros_msgs/ObjectState.h>

typedef unsigned char byte;
static int version;
static int depth, height, width;
static int size;
static byte *voxels = 0;
static float tx, ty, tz;
static float scale;

std::mutex pos_mutex;
float x_pos, y_pos, z_pos;
float x_or, y_or, z_or, w_or;
bool goodPos = false;

struct Voxel
{
  unsigned int x;
  unsigned int y;
  unsigned int z;
  Voxel() : x(0), y(0), z(0) {};
  Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z) : x(_x), y(_y), z(_z) {};
};

/*
// For debugging: Write the voxel indices to a file
void writeVoxelsToFile(const std::vector<Voxel>& voxels, const std::string& filename)
{
  std::ofstream* output = new std::ofstream(filename, std::ios::out);
  if (!output->good())
  {
    std::cerr << "Error: Could not open output file " << output << "! \n" << std::endl;
    exit(1);
  }
  for (size_t i = 0; i < voxels.size(); ++i)
  {
    // Write in order (X,Z,Y)
    *output << voxels.at(i).x << " "  << voxels.at(i).z << " "  << voxels.at(i).y << "\n";
  }
  output->close();
}
*/

template <typename PointT>
const bool loadPointCloud(const std::string& file_path,
                          typename pcl::PointCloud<PointT>& cloud_out)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path.c_str(), cloud_out) == -1)
  {
    PCL_ERROR("Failed to load PCD file \n");
    return false;
  }

  return true;
}

// Get the linear index into a 1D array of voxels,
// for a voxel at location (ix, iy, iz).
const unsigned int getLinearIndex(const Voxel& voxel, const int grid_size)
{
  return voxel.x * (grid_size * grid_size) + voxel.z * grid_size + voxel.y;
}

const Voxel getGridIndex(const pcl::PointXYZ& point, const pcl::PointXYZ& translate, const uint voxel_grid_size, const float scale)
{
  // Needs to be signed to prevent overflow, because index can
  // be slightly negative which then gets rounded to zero.
  const int i = std::round(static_cast<float>(voxel_grid_size)*((point.x - translate.x) / scale) - 0.5);
  const int j = std::round(static_cast<float>(voxel_grid_size)*((point.y - translate.y) / scale) - 0.5);
  const int k = std::round(static_cast<float>(voxel_grid_size)*((point.z - translate.z) / scale) - 0.5);
  return Voxel(i, j, k);  
}

// Format a float number to ensure it always has at least one decimal place
// 0 --> 0.0
// 1.1 --> 1.1
// 1.10 --> 1.1
const std::string formatFloat(const float value)
{
  std::stringstream ss;
  ss << std::setprecision(6) << std::fixed << value;
  std::string str;
  ss.str().swap(str);
  size_t last_zero_idx = str.find_last_not_of("0") + 1;
  if (last_zero_idx == str.length())
  {
    // No trailing zeros
    return str;
  }
  if (str[last_zero_idx - 1] == '.')
  {
    // Last zero is after decimal point
    last_zero_idx += 1;
  }
  str.resize(last_zero_idx);
  return str;
}

int read_binvox(std::string filespec)
{

  std::ifstream *input = new std::ifstream(filespec.c_str(), std::ios::in | std::ios::binary);

  //
  // read header
  //
  std::string line;
  *input >> line;  // #binvox
  if (line.compare("#binvox") != 0) {
    std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
    delete input;
    return 0;
  }
  *input >> version;
  std::cout << "reading binvox version " << version << std::endl;

  depth = -1;
  int done = 0;
  while(input->good() && !done) {
    *input >> line;
    if (line.compare("data") == 0) done = 1;
    else if (line.compare("dim") == 0) {
      *input >> depth >> height >> width;
    }
    else if (line.compare("translate") == 0) {
      *input >> tx >> ty >> tz;
    }
    else if (line.compare("scale") == 0) {
      *input >> scale;
    }
    else {
      std::cout << "  unrecognized keyword [" << line << "], skipping" << std::endl;
      char c;
      do {  // skip until end of line
        c = input->get();
      } while(input->good() && (c != '\n'));

    }
  }
  if (!done) {
    std::cout << "  error reading header" << std::endl;
    return 0;
  }
  if (depth == -1) {
    std::cout << "  missing dimensions in header" << std::endl;
    return 0;
  }

  size = width * height * depth;
  voxels = new byte[size];
  if (!voxels) {
    std::cout << "  error allocating memory" << std::endl;
    return 0;
  }

  //
  // read voxel data
  //
  byte value;
  byte count;
  int index = 0;
  int end_index = 0;
  int nr_voxels = 0;
  
  input->unsetf(std::ios::skipws);  // need to read every byte now (!)
  *input >> value;  // read the linefeed char

  while((end_index < size) && input->good()) {
    *input >> value >> count;

    if (input->good()) {
      end_index = index + count;
      if (end_index > size) return 0;
      for(int i=index; i < end_index; i++) voxels[i] = value;
      
      if (value) nr_voxels += count;
      index = end_index;
    }  // if file still ok
    
  }  // while

  input->close();
  std::cout << "  read " << nr_voxels << " voxels" << std::endl;

  return 1;
}


void cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
{
  // Declare Point Clouds
  pcl::PCLPointCloud2* pc2 = new pcl::PCLPointCloud2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // Concert the sensor_msgs to a Point Cloud
  pcl_conversions::toPCL(cloud_msg, *pc2);
  pcl::fromPCLPointCloud2(*pc2, *in_cloud);


  // Rotation stuff
  pos_mutex.lock();
  Eigen::Quaternionf quat (x_or, y_or, z_or, w_or);
  pos_mutex.unlock();
  Eigen::Matrix3f rotation = quat.conjugate().toRotationMatrix();
  Eigen::Affine3f transform (Eigen::Affine3f::Identity());
  transform.rotate(rotation);
  //pcl::transformPointCloud(*in_cloud, *cloud, transform);

  Eigen::Vector4f centroid (x_pos, y_pos, z_pos, 0.0);
  Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
  centroid_new.head<3>() = rotation * centroid.head<3>();
  transform.translation() = centroid.head<3>() - centroid_new.head<3>();
  pcl::transformPointCloud(*in_cloud, *cloud, transform);

  uint voxel_grid_size = 30; // Setting voxel_grid_size to default: 30

  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);

  // Calculate the scale factor so the longest side of the volume
  // is split into the desired number of voxels
  const float x_range = max_point.x - min_point.x;
  const float y_range = max_point.y - min_point.y;
  const float z_range = max_point.z - min_point.z;

  const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
  const float voxel_size = max_cloud_extent / (static_cast<float>(voxel_grid_size) - 1.0);
  std::cout << "voxel_size = " << voxel_size << std::endl;

  const float scale = (static_cast<float>(voxel_grid_size) * max_cloud_extent) / (static_cast<float>(voxel_grid_size) - 1.0);

  std::cout << "Bounding box: "
            << "[" << min_point.x << ", " << min_point.y << ", " << min_point.z << "] - "
            << "[" << max_point.x << ", " << max_point.y << ", " << max_point.z << "]" << std::endl;

  // Calculate the PointCloud's translation from the origin.
  // We need to subtract half the voxel size, because points
  // are located in the center of the voxel grid. 
  float tx = min_point.x - voxel_size / 2.0;
  float ty = min_point.y - voxel_size / 2.0;
  float tz = min_point.z - voxel_size / 2.0;

  // Hack, change -0.0 to 0.0
  const float epsilon = 0.0000001;
  if ((tx > -epsilon) && (tx < 0.0))
  {
    tx = -1.0 * tx;
  }
  if ((ty > -epsilon) && (ty < 0.0))
  {
    ty = -1.0 * ty;
  }
  if ((tz > -epsilon) && (tz < 0.0))
  {
    tz = -1.0 * tz;
  }

  const pcl::PointXYZ translate(tx, ty, tz);
  std::cout << "Normalization transform: (1) translate ["
            << formatFloat(translate.x) << ", " << formatFloat(translate.y) << ", " << formatFloat(translate.z) << "], " << std::endl;
  std::cout << "                         (2) scale " << scale << std::endl;

  const unsigned int num_voxels = voxel_grid_size * voxel_grid_size * voxel_grid_size;

  //Finding center!
  pos_mutex.lock();
  pcl::PointXYZ center (x_pos, y_pos, z_pos);
  pos_mutex.unlock();
  const Voxel center_voxel = getGridIndex(center, translate, voxel_grid_size, scale);
  printf("Center: %d, %d, %d\n", center_voxel.x, center_voxel.y, center_voxel.z);

  // Voxelize the PointCloud into a linear array
  boost::dynamic_bitset<> voxels_bitset(num_voxels);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
  {
    Voxel voxel = getGridIndex(*it, translate, voxel_grid_size, scale);
    voxel.x = voxel.x + (15 - center_voxel.x);
    voxel.y = voxel.y + (15 - center_voxel.y);
    voxel.z = voxel.z + (15 - center_voxel.z);
    if(voxel.x < 0 || voxel.x >= voxel_grid_size || voxel.y < 0 || voxel.y >= voxel_grid_size || voxel.z < 0 || voxel.z >= voxel_grid_size)
      continue;
    const unsigned int idx = getLinearIndex(voxel, voxel_grid_size);
    voxels_bitset[idx] = 1;
  }


  /*
  // For debugging: Write voxel indices to a file
  std::vector<Voxel> voxels; // for debugging
  for (size_t i = 0; i < voxels_bitset.size(); ++i)
  {
    if (voxels_bitset[i] == 1)
    {
      const int voxel_grid_width = voxel_grid_size;
      const int voxel_grid_height = voxel_grid_size;
      const int idx = static_cast<int>(i);
      const float ix = static_cast<float>(idx / (voxel_grid_width * voxel_grid_height));
      const float iy = static_cast<float>(idx % voxel_grid_size);
      const float iz = static_cast<float>((idx / voxel_grid_width) % voxel_grid_height);
      voxels.push_back( Voxel(ix, iy, iz) );
    }
  }
  writeVoxelsToFile(voxels, "voxels_from_pcd.txt");
  */

  const std::string output_file("tmp.binvox");
  std::ofstream* output = new std::ofstream(output_file, std::ios::out | std::ios::binary);
  if (!output->good())
  {
    std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
    exit(1);
  }

  // Write the binvox file using run-length encoding
  // where each pair of bytes is of the format (run value, run length)
  *output << "#binvox 1\n";
  *output << "dim " << voxel_grid_size << " " << voxel_grid_size << " " << voxel_grid_size << "\n";
  *output << "translate " << formatFloat(translate.x) << " " << formatFloat(translate.y) << " " << formatFloat(translate.z) << "\n";
  *output << "scale " << scale << "\n";
  *output << "data\n";
  unsigned int run_value = voxels_bitset[0];
  unsigned int run_length = 0;
  for (size_t i = 0; i < num_voxels; ++i)
  {
    if (voxels_bitset[i] == run_value)
    {
      // This is a run (repeated bit value)
      run_length++;
      if (run_length == 255)
      {
        *output << static_cast<char>(run_value);
        *output << static_cast<char>(run_length);
        run_length = 0;
      }
    }
    else
    {
      // End of a run
      *output << static_cast<char>(run_value);
      *output << static_cast<char>(run_length);
      run_value = voxels_bitset[i];
      run_length = 1;
    }
  }
  if (run_length > 0)
  {
    *output << static_cast<char>(run_value);
    *output << static_cast<char>(run_length);
  }
  output->close();

  std::cout << "done" << std::endl << std::endl;
  //rename("tmp.txt", "out.txt");

  read_binvox("tmp.binvox");
  std::ofstream *out = new std::ofstream("tmp.txt");
  if(!out->good()) {
    std::cout << "Error opening [voxels.txt]" << std::endl << std::endl;
    exit(1);
  }
  for(int i=0; i < size; i++) {
    *out << (char) (voxels[i] + '0') << " ";
    if (((i + 1) % width) == 0) *out << std::endl;
  }
  out->close();
  rename("tmp.txt", "voxels.txt");

}

void state_cb (const dbot_ros_msgs::ObjectState& state_msg)
{
  std::lock_guard<std::mutex> lock(pos_mutex);
  //ROS_INFO("Got a state message!\n");
  printf("Received pose %f, %f, %f\n", state_msg.pose.pose.position.x, state_msg.pose.pose.position.y, state_msg.pose.pose.position.z);
  x_pos = state_msg.pose.pose.position.x;
  y_pos = state_msg.pose.pose.position.y;
  z_pos = state_msg.pose.pose.position.z;
  x_or = state_msg.pose.pose.orientation.x;
  y_or = state_msg.pose.pose.orientation.y;
  z_or = state_msg.pose.pose.orientation.z;
  w_or = state_msg.pose.pose.orientation.w;
  goodPos = true;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcd2binvox");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("out_cloud", 1, cloud_cb);

  ros::Subscriber sub_state = nh.subscribe("particle_tracker/object_state", 1, state_cb);

  // Spin
  ros::spin ();

  return 0;
}


// int main(int argc, char **argv)
// {
//   if (argc < 4)
//   {
//     pcl::console::print_error("Syntax is: %s -d <voxel_grid_size [32 to 1024]> input.pcd output.binvox \n", argv[0]);
//     return -1;
//   }

//   // Parse the command line arguments for .pcd and .ply files
//   std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
//   std::vector<int> binvox_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".binvox");
//   if (pcd_file_indices.size() != 1 || binvox_file_indices.size() != 1)
//   {
//     pcl::console::print_error("Need one input PCD file and one output Binvox file. \n");
//     return -1;
//   }

//   // In binvox, default is 256, max 1024
//   uint voxel_grid_size;
//   pcl::console::parse_argument(argc, argv, "-d", voxel_grid_size);
//   std::cout << "Voxel grid size: " << voxel_grid_size << std::endl;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//   const std::string input_file(argv[pcd_file_indices[0]]);
//   if (!loadPointCloud<pcl::PointXYZ>(input_file, *cloud))
//   {
//     return -1;
//   }

//   pcl::PointXYZ min_point;
//   pcl::PointXYZ max_point;
//   pcl::getMinMax3D(*cloud, min_point, max_point);

//   // Calculate the scale factor so the longest side of the volume
//   // is split into the desired number of voxels
//   const float x_range = max_point.x - min_point.x;
//   const float y_range = max_point.y - min_point.y;
//   const float z_range = max_point.z - min_point.z;

//   const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
//   const float voxel_size = max_cloud_extent / (static_cast<float>(voxel_grid_size) - 1.0);
//   std::cout << "voxel_size = " << voxel_size << std::endl;

//   const float scale = (static_cast<float>(voxel_grid_size) * max_cloud_extent) / (static_cast<float>(voxel_grid_size) - 1.0);

//   std::cout << "Bounding box: "
//             << "[" << min_point.x << ", " << min_point.y << ", " << min_point.z << "] - "
//             << "[" << max_point.x << ", " << max_point.y << ", " << max_point.z << "]" << std::endl;

//   // Calculate the PointCloud's translation from the origin.
//   // We need to subtract half the voxel size, because points
//   // are located in the center of the voxel grid. 
//   float tx = min_point.x - voxel_size / 2.0;
//   float ty = min_point.y - voxel_size / 2.0;
//   float tz = min_point.z - voxel_size / 2.0;
//   // Hack, change -0.0 to 0.0
//   const float epsilon = 0.0000001;
//   if ((tx > -epsilon) && (tx < 0.0))
//   {
//     tx = -1.0 * tx;
//   }
//   if ((ty > -epsilon) && (ty < 0.0))
//   {
//     ty = -1.0 * ty;
//   }
//   if ((tz > -epsilon) && (tz < 0.0))
//   {
//     tz = -1.0 * tz;
//   }
//   const pcl::PointXYZ translate(tx, ty, tz);
//   std::cout << "Normalization transform: (1) translate ["
//             << formatFloat(translate.x) << ", " << formatFloat(translate.y) << ", " << formatFloat(translate.z) << "], " << std::endl;
//   std::cout << "                         (2) scale " << scale << std::endl;

//   const unsigned int num_voxels = voxel_grid_size * voxel_grid_size * voxel_grid_size;

//   // Voxelize the PointCloud into a linear array
//   boost::dynamic_bitset<> voxels_bitset(num_voxels);
//   for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
//   {
//     const Voxel voxel = getGridIndex(*it, translate, voxel_grid_size, scale);
//     const unsigned int idx = getLinearIndex(voxel, voxel_grid_size);
//     voxels_bitset[idx] = 1;
//   }

//   /*
//   // For debugging: Write voxel indices to a file
//   std::vector<Voxel> voxels; // for debugging
//   for (size_t i = 0; i < voxels_bitset.size(); ++i)
//   {
//     if (voxels_bitset[i] == 1)
//     {
//       const int voxel_grid_width = voxel_grid_size;
//       const int voxel_grid_height = voxel_grid_size;
//       const int idx = static_cast<int>(i);
//       const float ix = static_cast<float>(idx / (voxel_grid_width * voxel_grid_height));
//       const float iy = static_cast<float>(idx % voxel_grid_size);
//       const float iz = static_cast<float>((idx / voxel_grid_width) % voxel_grid_height);
//       voxels.push_back( Voxel(ix, iy, iz) );
//     }
//   }
//   writeVoxelsToFile(voxels, "voxels_from_pcd.txt");
//   */

//   const std::string output_file(argv[binvox_file_indices[0]]);
//   std::ofstream* output = new std::ofstream(output_file, std::ios::out | std::ios::binary);
//   if (!output->good())
//   {
//     std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
//     exit(1);
//   }

//   // Write the binvox file using run-length encoding
//   // where each pair of bytes is of the format (run value, run length)
//   *output << "#binvox 1\n";
//   *output << "dim " << voxel_grid_size << " " << voxel_grid_size << " " << voxel_grid_size << "\n";
//   *output << "translate " << formatFloat(translate.x) << " " << formatFloat(translate.y) << " " << formatFloat(translate.z) << "\n";
//   *output << "scale " << scale << "\n";
//   *output << "data\n";
//   unsigned int run_value = voxels_bitset[0];
//   unsigned int run_length = 0;
//   for (size_t i = 0; i < num_voxels; ++i)
//   {
//     if (voxels_bitset[i] == run_value)
//     {
//       // This is a run (repeated bit value)
//       run_length++;
//       if (run_length == 255)
//       {
//         *output << static_cast<char>(run_value);
//         *output << static_cast<char>(run_length);
//         run_length = 0;
//       }
//     }
//     else
//     {
//       // End of a run
//       *output << static_cast<char>(run_value);
//       *output << static_cast<char>(run_length);
//       run_value = voxels_bitset[i];
//       run_length = 1;
//     }
//   }
//   if (run_length > 0)
//   {
//     *output << static_cast<char>(run_value);
//     *output << static_cast<char>(run_length);
//   }
//   output->close();

//   std::cout << "done" << std::endl << std::endl;
//   return 0;
// }
