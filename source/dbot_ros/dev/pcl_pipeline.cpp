#include <ros/ros.h>
#include <ros/package.h>

#include <mutex>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip> // setprecision
#include <stdlib.h>
#include <cstdio>
#include <dirent.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/dynamic_bitset.hpp>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/Vertices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <dbot_ros_msgs/ObjectState.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/SolidPrimitive.h>
#include <resource_retriever/retriever.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarker.h>

#define PI 3.14159265

ros::Publisher pub, bb_pub, mesh_pub;
std::mutex pos_mutex;

float x_pos, y_pos, z_pos, radius;
geometry_msgs::Pose original_pose;
bool goodPos = false;
Eigen::Vector4f minPoint, maxPoint;

Eigen::Affine3f cropAffine;
Eigen::Quaternionf full_rotation;


typedef unsigned char byte;
static int version;
static int depth, height, width;
static int size;
static byte *voxels = 0;
static float tx, ty, tz;
static float scale;

struct Voxel
{
  unsigned int x;
  unsigned int y;
  unsigned int z;
  Voxel() : x(0), y(0), z(0) {};
  Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z) : x(_x), y(_y), z(_z) {};
};

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
  //std::cout << "reading binvox version " << version << std::endl;

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
  //std::cout << "  read " << nr_voxels << " voxels" << std::endl;

  return 1;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //std::lock_guard<std::mutex> lock(pos_mutex);
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloud_filteredPtr(cloud_filtered);
  pcl::PCLPointCloud2 cloud_filtered2;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  if(goodPos) {
    // Set up the bouning box for the Crop filter
    //float hack_radius = 0.8 * radius;
    minPoint[0] = -radius;
    minPoint[1] = -radius;
    minPoint[2] = -radius;
    maxPoint[0] = radius;
    maxPoint[1] = radius;
    maxPoint[2] = radius;
    //printf("Radius: %f\n", radius);
    // Filter
    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    //cropFilter.setTransform(cropAffine);
    pos_mutex.lock();
    cropFilter.setTranslation(Eigen::Vector3f(x_pos, y_pos, z_pos));
    cropFilter.setRotation(cropAffine.linear().eulerAngles(0, 1, 2));
    pos_mutex.unlock();
    cropFilter.filter(*cloud_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_filteredPtr);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(cloud_filtered2);
  }
  else {
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered2);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 out_cloud;
  pcl_conversions::fromPCL(cloud_filtered2, out_cloud); //moveFromPCL call faster?

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered2, *in_cloud);

  // Rotation stuff
  pos_mutex.lock();
  Eigen::Quaternionf quat = full_rotation;
  pos_mutex.unlock();
  Eigen::Matrix3f rotation = quat.conjugate().toRotationMatrix();
  Eigen::Affine3f transform (Eigen::Affine3f::Identity());
  transform.rotate(rotation);
  //pcl::transformPointCloud(*in_cloud, *cloud, transform);

  pos_mutex.lock();
  Eigen::Vector4f centroid (x_pos, y_pos, z_pos, 0.0);
  pos_mutex.unlock();
  Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
  centroid_new.head<3>() = rotation * centroid.head<3>();
  transform.translation() = centroid.head<3>() - centroid_new.head<3>();
  pcl::transformPointCloud(*in_cloud, *trans_cloud, transform);

  uint voxel_grid_size = 30; // Setting voxel_grid_size to default: 30

  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::getMinMax3D(*trans_cloud, min_point, max_point);

  // Calculate the scale factor so the longest side of the volume
  // is split into the desired number of voxels
  const float x_range = max_point.x - min_point.x;
  const float y_range = max_point.y - min_point.y;
  const float z_range = max_point.z - min_point.z;

  const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
  const float voxel_size = max_cloud_extent / (static_cast<float>(voxel_grid_size) - 1.0);
  //std::cout << "voxel_size = " << voxel_size << std::endl;

  const float scale = (static_cast<float>(voxel_grid_size) * max_cloud_extent) / (static_cast<float>(voxel_grid_size) - 1.0);

  // std::cout << "Bounding box: "
  //           << "[" << min_point.x << ", " << min_point.y << ", " << min_point.z << "] - "
  //           << "[" << max_point.x << ", " << max_point.y << ", " << max_point.z << "]" << std::endl;

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
  // std::cout << "Normalization transform: (1) translate ["
  //           << formatFloat(translate.x) << ", " << formatFloat(translate.y) << ", " << formatFloat(translate.z) << "], " << std::endl;
  //std::cout << "                         (2) scale " << scale << std::endl;

  const unsigned int num_voxels = voxel_grid_size * voxel_grid_size * voxel_grid_size;

  //Finding center!
  pos_mutex.lock();
  pcl::PointXYZ center (x_pos, y_pos, z_pos);
  pos_mutex.unlock();
  const Voxel center_voxel = getGridIndex(center, translate, voxel_grid_size, scale);
  //printf("Center: %d, %d, %d\n", center_voxel.x, center_voxel.y, center_voxel.z);

  // Voxelize the PointCloud into a linear array
  boost::dynamic_bitset<> voxels_bitset(num_voxels);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = trans_cloud->begin(); it != trans_cloud->end(); ++it)
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

  std::string dbot_ros_path = ros::package::getPath("dbot_ros");
  const std::string output_file(dbot_ros_path + "/tmp.binvox");
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

  //std::cout << "done" << std::endl << std::endl;
  //rename("tmp.txt", "out.txt");

  read_binvox(dbot_ros_path + "/tmp.binvox");
  std::ofstream *out = new std::ofstream(dbot_ros_path + "/tmp.txt");
  if(!out->good()) {
    std::cout << "Error opening [voxels.txt]" << std::endl << std::endl;
    exit(1);
  }
  for(int i=0; i < size; i++) {
    *out << (char) (voxels[i] + '0') << " ";
    if (((i + 1) % width) == 0) *out << std::endl;
  }
  out->close();
  rename(&(dbot_ros_path + "/tmp.txt")[0], &(dbot_ros_path + "/voxels.txt")[0]);


  // Publish the data
  pub.publish (out_cloud);
}

void state_cb (const dbot_ros_msgs::ObjectState& state_msg)
{
  std::lock_guard<std::mutex> lock(pos_mutex);
  x_pos = state_msg.pose.pose.position.x;
  y_pos = state_msg.pose.pose.position.y;
  z_pos = state_msg.pose.pose.position.z;
  goodPos = true;
}

void intmodel_cb (const visualization_msgs::InteractiveMarkerInit& intmodel_msg)
{
  //std::lock_guard<std::mutex> lock(pos_mutex);
  original_pose.orientation.x = intmodel_msg.markers[0].pose.orientation.x;
  original_pose.orientation.y = intmodel_msg.markers[0].pose.orientation.y;
  original_pose.orientation.z = intmodel_msg.markers[0].pose.orientation.z;
  original_pose.orientation.w = intmodel_msg.markers[0].pose.orientation.w;
  printf("Original Pose: %f, %f, %f, %f\n", intmodel_msg.markers[0].pose.orientation.x, intmodel_msg.markers[0].pose.orientation.y, intmodel_msg.markers[0].pose.orientation.z, intmodel_msg.markers[0].pose.orientation.w);
}

void model_cb (const visualization_msgs::Marker& model_msg)
{
  if (model_msg.type == model_msg.MESH_RESOURCE) {
    shapes::Shape* mesh = shapes::createMeshFromResource("package://object_meshes/object_models/oil_bottle.obj");
    const bodies::ConvexMesh* cm = new bodies::ConvexMesh(mesh);
    bodies::BoundingCylinder cyl;
    cm->computeBoundingCylinder(cyl);
    radius = cyl.length / 2.0;

    Eigen::Quaternionf og_rotation(original_pose.orientation.w, original_pose.orientation.x, 
                                   original_pose.orientation.y, original_pose.orientation.z); //NOTE: Eigen::Quaternion takes w,x,y,z
    Eigen::Quaternionf m_rotation(model_msg.pose.orientation.w, model_msg.pose.orientation.x, 
                                  model_msg.pose.orientation.y, model_msg.pose.orientation.z);
    Eigen::Quaternionf composed = og_rotation * m_rotation;
    Eigen::Vector3f trans(model_msg.pose.position.x, model_msg.pose.position.y, model_msg.pose.position.z);
    Eigen::Affine3f crop = Eigen::Affine3f::Identity();
    crop.translation() = trans;
    crop.linear() = composed.toRotationMatrix(); // * Eigen::Scaling(radius)

    pos_mutex.lock();
    cropAffine = crop;
    full_rotation = composed;
    pos_mutex.unlock();

    // Publish the mesh reconstruction marker
    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "/camera_depth_optical_frame"; //SPECIFIC (hardcoded)
    mesh_marker.header.stamp = ros::Time::now();
    mesh_marker.ns = "recon_mesh";
    mesh_marker.id = 1;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::MODIFY;

    mesh_marker.pose.position.x = model_msg.pose.position.x;
    mesh_marker.pose.position.y = model_msg.pose.position.y;
    mesh_marker.pose.position.z = model_msg.pose.position.z;
    mesh_marker.pose.orientation.x = composed.x();
    mesh_marker.pose.orientation.y = composed.y();
    mesh_marker.pose.orientation.z = composed.z();
    mesh_marker.pose.orientation.w = composed.w();

    mesh_marker.scale.x = radius * 2.0;
    mesh_marker.scale.y = radius * 2.0;
    mesh_marker.scale.z = radius * 2.0;
    //printf("Radius * 2.0 = %f\n", radius * 2.0);
    mesh_marker.color.r = 0.0;
    mesh_marker.color.g = 1.0;
    mesh_marker.color.b = 1.0;
    mesh_marker.color.a = 1.0;

    // Hack to continuously update the mesh file (need to have the file name keep changing)
    int max_file_num = 0;
    std::string stl_file = "recon_0.stl";
    DIR* dir;
    struct dirent* ent;
    std::string path = ros::package::getPath("dbot_ros");
    path += "/../dbot_getting_started/object_meshes/object_models";
    if((dir = opendir(&path[0])) != NULL) {
      while((ent = readdir(dir)) != NULL) {
        std::string file (ent->d_name);
        //std::cout << file << std::endl;
        std::size_t found = file.find(".stl");
        if(found != std::string::npos) {
          int i;
          sscanf(&file[0], "recon_%d.stl", &i);
          if (i > max_file_num) {
            max_file_num = i;
            stl_file = file;
          }
        }
      }
      closedir(dir);
    }
    else {
      std::cout << "Error opening directory!" << std::endl;
    }

    std::ostringstream oss;
    oss << "package://object_meshes/object_models/" << stl_file;
    mesh_marker.mesh_resource = oss.str();
    mesh_pub.publish(mesh_marker); //publish the mesh 
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  x_pos = y_pos = z_pos = 0.0;

  // Create a ROS subscribers
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);
  ros::Subscriber sub_state = nh.subscribe("particle_tracker/object_state", 1, state_cb);
  ros::Subscriber sub_intmodel = nh.subscribe("interactive_marker_initializer/update_full", 1, intmodel_cb);
  ros::Subscriber sub_model = nh.subscribe("particle_tracker/object_model", 1, model_cb);

  // Create a ROS publisher for the output point cloud and reconstructed mesh
  pub = nh.advertise<sensor_msgs::PointCloud2> ("out_cloud", 1);
  mesh_pub = nh.advertise<visualization_msgs::Marker> ("recon_mesh", 1);

  // Spin
  ros::spin ();
}