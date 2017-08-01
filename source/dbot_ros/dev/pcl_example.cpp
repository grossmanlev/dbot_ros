#include <ros/ros.h>
#include <mutex>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <dirent.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/Vertices.h>

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

#define BOUNDING_SIZE 0.25
#define PI 3.14159265

ros::Publisher pub, bb_pub, mesh_pub;
std::mutex pos_mutex;

float x_pos, y_pos, z_pos, radius;
geometry_msgs::Pose original_pose;
bool goodPos = false;
Eigen::Vector4f minPoint, maxPoint;

Eigen::Affine3f cropAffine;

// bool getBoundingCylinderOfMesh(std::string mesh_file, shapes::Shape &mesh, bodies::BoundingCylinder &cyl) // adapted from ROS user Benjamin Cohen
// {
//   //mesh_file = "package://object_meshes/object_models/oil_bottle.obj";
//   //if (!mesh_file.empty())
//   //  return false;
    
//   shapes::Shape* tmp = shapes::createMeshFromResource("package://object_meshes/object_models/oil_bottle.obj");
//   mesh = *tmp;
//   tmp->print();
//   const bodies::ConvexMesh *cm = new bodies::ConvexMesh(tmp);
//   bodies::BoundingCylinder bc;
//   cm->computeBoundingCylinder(bc);
//   ROS_INFO("Bounding sphere has radius: %0.3f and length %0.3f\n", bc.radius, bc.length);
//   cyl = bc;
//   printf("HERE\n");
//   return true;
// }

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::lock_guard<std::mutex> lock(pos_mutex);
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloud_filteredPtr(cloud_filtered);
  pcl::PCLPointCloud2 cloud_filtered2;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // if(goodPos) {
  //   pcl::CropHull<pcl::PointXYZ> cropHullFilter;
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints (new pcl::PointCloud<pcl::PointXYZ>);
  //   std::vector<pcl::Vertices> hullPolygons;
  //   pcl::PointXYZ p;
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr hullFiltered (new pcl::PointCloud<pcl::PointXYZ>);

  //   // Convert to PCL data type
  //   pcl_conversions::toPCL(*cloud_msg, *cloud);

  //   hullCloud->clear();
  //   // Create sphere points
  //   printf("Before sphere\n");
  //   for(float phi = 0.; phi < PI; phi += PI/5.) {
  //     for(float theta = 0.; theta < 2.0*PI; theta += PI/5.) {
  //       p.x = radius * cos(theta) * sin(phi) + x_pos;
  //       p.y = radius * sin(theta) * sin(phi) + y_pos;
  //       p.z = radius * cos(phi) + z_pos;
  //       hullCloud->push_back(p);
  //     }
  //   }
  //   printf("After sphere\n");
  //   pcl::ConvexHull<pcl::PointXYZ> cHull;
  //   cHull.setInputCloud(hullCloud);
  //   cHull.reconstruct(*hullPoints, hullPolygons);
  //   printf("After reconstruct\n");

  //   cropHullFilter.setHullIndices(hullPolygons);
  //   cropHullFilter.setHullCloud(hullPoints);

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::fromPCLPointCloud2(*cloud, *inputCloud);
  //   cropHullFilter.setInputCloud(inputCloud);
  //   cropHullFilter.filter(*hullFiltered);
  //   printf("After filtered\n");
  //   pcl::toPCLPointCloud2(*hullFiltered, *cloud_filtered);
  // }



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
    cropFilter.setTranslation(Eigen::Vector3f(x_pos, y_pos, z_pos));
    cropFilter.setRotation(cropAffine.linear().eulerAngles(0, 1, 2));
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

  // Publish the data
  pub.publish (out_cloud);
}

void state_cb (const dbot_ros_msgs::ObjectState& state_msg)
{
  std::lock_guard<std::mutex> lock(pos_mutex);
  //ROS_INFO("Got a state message!\n");
  //printf("Received pose %f, %f, %f\n", state_msg.pose.pose.position.x, state_msg.pose.pose.position.y, state_msg.pose.pose.position.z);
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
    //mesh->print();
    const bodies::ConvexMesh* cm = new bodies::ConvexMesh(mesh);
    bodies::BoundingCylinder cyl;
    cm->computeBoundingCylinder(cyl);
    //ROS_INFO("Bounding sphere has radius: %0.3f and length %0.3f\n", cyl.radius, cyl.length);
    radius = cyl.length / 2.0;


    Eigen::Quaternionf og_rotation(original_pose.orientation.w, original_pose.orientation.x, 
                                   original_pose.orientation.y, original_pose.orientation.z); //NOTE: Eigen::Quaternion takes w,x,y,z
    Eigen::Quaternionf m_rotation(model_msg.pose.orientation.w, model_msg.pose.orientation.x, 
                                  model_msg.pose.orientation.y, model_msg.pose.orientation.z);
    Eigen::Quaternionf composed = og_rotation * m_rotation;
    // Eigen::Quaternionf q1 = og_rotation; Eigen::Quaternionf q2 = m_rotation;
    // composed.setIdentity();

    // composed.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    // composed.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    //printf("Original Pose: %f, %f, %f, %f\n", og_rotation.x(), og_rotation.y(), og_rotation.z(), og_rotation.w());
    //printf("Marker Pose: %f, %f, %f, %f\n", m_rotation.x(), m_rotation.y(), m_rotation.z(), m_rotation.w());

    //Eigen::Quaterniond quat(cyl.pose.rotation());
    //Eigen::Translation<double, 3> trans(cyl.pose.translation());
    Eigen::Vector3f trans(model_msg.pose.position.x, model_msg.pose.position.y, model_msg.pose.position.z);

    Eigen::Affine3f crop = Eigen::Affine3f::Identity();
    //Eigen::Translation3f(trans) * Eigen::AngleAxisf(composed) * Eigen::Scaling(0.1);
    crop.translation() = trans;
    crop.linear() = composed.toRotationMatrix(); // * Eigen::Scaling(radius)

    pos_mutex.lock();
    cropAffine = crop;
    pos_mutex.unlock();
    //crop.linear() = composed.toRotationMatrix() * Eigen::Scaling(radius);
    //crop.translation() = trans;


    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "/camera_depth_optical_frame"; //SPECIFIC (hardcoded)
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "bounding_box";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::SPHERE;
    // marker.action = visualization_msgs::Marker::ADD;

    // printf("Pose: %f, %f, %f\n", model_msg.pose.position.x, model_msg.pose.position.y, model_msg.pose.position.z);
    // marker.pose.position.x = model_msg.pose.position.x;
    // marker.pose.position.y = model_msg.pose.position.y;
    // marker.pose.position.z = model_msg.pose.position.z;
    // marker.pose.orientation.x = composed.x();
    // marker.pose.orientation.y = composed.y();
    // marker.pose.orientation.z = composed.z();
    // marker.pose.orientation.w = composed.w();

    // marker.scale.x = cyl.length;
    // marker.scale.y = cyl.length;
    // marker.scale.z = cyl.length;
    // marker.color.r = 0.0f;
    // marker.color.g = 0.0f;
    // marker.color.b = 1.0f;
    // marker.color.a = 1.0;

    // marker.lifetime = ros::Duration();
    // bb_pub.publish(marker);


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
    mesh_marker.color.r = 0.0;
    mesh_marker.color.g = 1.0;
    mesh_marker.color.b = 1.0;
    mesh_marker.color.a = 1.0;

    // Hack to continuously update the mesh file (need to have the file name keep changing)
    int max_file_num = 0;
    std::string stl_file = "recon_0.stl";
    DIR* dir;
    struct dirent* ent;
    if((dir = opendir("src/dbot_getting_started/object_meshes/object_models")) != NULL) {
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
    //recon_stl_num = (recon_stl_num + 1) % 2;
    mesh_pub.publish(mesh_marker); //publish the mesh 
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  x_pos = y_pos = z_pos = 0.0;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

  // Create a ROS subscriber for the object_state publisher
  ros::Subscriber sub_state = nh.subscribe("particle_tracker/object_state", 1, state_cb);

  ros::Subscriber sub_intmodel = nh.subscribe("interactive_marker_initializer/update_full", 1, intmodel_cb);
  ros::Subscriber sub_model = nh.subscribe("particle_tracker/object_model", 1, model_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("out_cloud", 1);

  //bb_pub = nh.advertise<visualization_msgs::Marker> ("bounding_box", 1);

  mesh_pub = nh.advertise<visualization_msgs::Marker> ("recon_mesh", 1);

  // Spin
  ros::spin ();
}