#include <ros/ros.h>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

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

#include <iostream>

#define BOUNDING_SIZE 0.25

ros::Publisher pub;
ros::Publisher bb_pub;
std::mutex pos_mutex;

float x_pos, y_pos, z_pos;
geometry_msgs::Pose original_pose;
bool goodPos = false;
Eigen::Vector4f minPoint, maxPoint;

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
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  if(goodPos) {
    // Set up the bouning box for the Crop filter
    minPoint[0] = x_pos - BOUNDING_SIZE;
    minPoint[1] = y_pos - BOUNDING_SIZE;
    minPoint[2] = z_pos - BOUNDING_SIZE;
    maxPoint[0] = x_pos + BOUNDING_SIZE;
    maxPoint[1] = y_pos + BOUNDING_SIZE;
    maxPoint[2] = z_pos + BOUNDING_SIZE;
    // Filter
    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter(cloud_filtered);
  }
  else {
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);
  }


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output); //moveFromPCL call faster?

  // Publish the data
  pub.publish (output);
}

void state_cb (const dbot_ros_msgs::ObjectState& state_msg)
{
  std::lock_guard<std::mutex> lock(pos_mutex);
  //ROS_INFO("Got a state message!\n");
  printf("Received pose %f, %f, %f\n", state_msg.pose.pose.position.x, state_msg.pose.pose.position.y, state_msg.pose.pose.position.z);
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
    mesh->print();
    const bodies::ConvexMesh* cm = new bodies::ConvexMesh(mesh);
    bodies::BoundingCylinder cyl;
    cm->computeBoundingCylinder(cyl);
    ROS_INFO("Bounding sphere has radius: %0.3f and length %0.3f\n", cyl.radius, cyl.length);


    Eigen::Quaternionf og_rotation(original_pose.orientation.x, original_pose.orientation.y, 
                                   original_pose.orientation.z, original_pose.orientation.w);
    Eigen::Quaternionf m_rotation(model_msg.pose.orientation.x, model_msg.pose.orientation.y, 
                                  model_msg.pose.orientation.z, model_msg.pose.orientation.w);
    Eigen::Quaternionf composed;
    Eigen::Quaternionf q1 = og_rotation; Eigen::Quaternionf q2 = m_rotation;
    composed.setIdentity();

    composed.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    composed.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    printf("Original Pose: %f, %f, %f, %f\n", og_rotation.x(), og_rotation.y(), og_rotation.z(), og_rotation.w());
    printf("Marker Pose: %f, %f, %f, %f\n", m_rotation.x(), m_rotation.y(), m_rotation.z(), m_rotation.w());

    Eigen::Quaterniond quat(cyl.pose.rotation());
    Eigen::Translation<double,3> trans(cyl.pose.translation());

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_depth_optical_frame"; //SPECIFIC (hardcoded)
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    printf("Pose: %f, %f, %f\n", model_msg.pose.position.x, model_msg.pose.position.y, model_msg.pose.position.z);
    marker.pose.position.x = model_msg.pose.position.x;
    marker.pose.position.y = model_msg.pose.position.y;
    marker.pose.position.z = model_msg.pose.position.z;
    marker.pose.orientation.x = composed.x();
    marker.pose.orientation.y = composed.y();
    marker.pose.orientation.z = composed.z();
    marker.pose.orientation.w = composed.w();

    marker.scale.x = cyl.radius;
    marker.scale.y = cyl.radius;
    marker.scale.z = cyl.length;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    bb_pub.publish(marker);
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  x_pos = y_pos = z_pos = 0.0;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS subscriber for the object_state publisher
  ros::Subscriber sub_state = nh.subscribe("particle_tracker/object_state", 1, state_cb);

  ros::Subscriber sub_intmodel = nh.subscribe("interactive_marker_initializer/update_full", 1, intmodel_cb);
  ros::Subscriber sub_model = nh.subscribe("particle_tracker/object_model", 1, model_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  bb_pub = nh.advertise<visualization_msgs::Marker> ("bounding_box", 1);

  // Spin
  ros::spin ();
}