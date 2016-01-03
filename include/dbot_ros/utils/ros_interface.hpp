/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file ros_interface.hpp
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <string>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <cv.h>

// to avoid stupid typedef conflict
#define uint64 enchiladisima
#include <cv_bridge/cv_bridge.h>
#undef uint64

#include <sensor_msgs/Image.h>

#include <osr/pose_vector.hpp>
#include <osr/pose_velocity_vector.hpp>

namespace ri
{
/**
 * \brief Converts a ros pose message to osr::PoseVector
 */
inline osr::PoseVector to_pose_vector(const geometry_msgs::Pose& ros_pose)
{
    Eigen::Vector3d p;
    Eigen::Quaternion<double> q;
    p[0] = ros_pose.position.x;
    p[1] = ros_pose.position.y;
    p[2] = ros_pose.position.z;
    q.w() = ros_pose.orientation.w;
    q.x() = ros_pose.orientation.x;
    q.y() = ros_pose.orientation.y;
    q.z() = ros_pose.orientation.z;

    osr::PoseVector pose;
    pose.position() = p;
    pose.orientation().quaternion(q);

    return pose;
}

/**
 * \brief Converts a ros pose message to osr::PoseVelocityVector
 */
inline osr::PoseVelocityVector to_pose_velocity_vector(
    const geometry_msgs::Pose& ros_pose)
{
    Eigen::Vector3d p;
    Eigen::Quaternion<double> q;
    p[0] = ros_pose.position.x;
    p[1] = ros_pose.position.y;
    p[2] = ros_pose.position.z;
    q.w() = ros_pose.orientation.w;
    q.x() = ros_pose.orientation.x;
    q.y() = ros_pose.orientation.y;
    q.z() = ros_pose.orientation.z;

    osr::PoseVelocityVector pose;
    pose.position() = p;
    pose.orientation().quaternion(q);

    return pose;
}



template <typename Parameter>
void ReadParameter(const std::string& path,
                   Parameter& parameter,
                   ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter = Parameter(ros_parameter);
}

template <>
void ReadParameter<std::vector<std::string>>(
    const std::string& path,
    std::vector<std::string>& parameter,
    ros::NodeHandle node_handle);

template <>
void ReadParameter<std::vector<double>>(const std::string& path,
                                        std::vector<double>& parameter,
                                        ros::NodeHandle node_handle);

template <>
void ReadParameter<std::vector<std::vector<size_t>>>(
    const std::string& path,
    std::vector<std::vector<size_t>>& parameter,
    ros::NodeHandle node_handle);

template <typename Scalar>
Eigen::Matrix<Scalar, -1, -1> Ros2Eigen(const sensor_msgs::Image& ros_image,
                                        const size_t& n_downsampling = 1)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(ros_image)->image;

    size_t n_rows = cv_image.rows / n_downsampling;
    size_t n_cols = cv_image.cols / n_downsampling;
    Eigen::Matrix<Scalar, -1, -1> eigen_image(n_rows, n_cols);
    for (size_t row = 0; row < n_rows; row++)
        for (size_t col = 0; col < n_cols; col++)
            eigen_image(row, col) =
                cv_image.at<float>(row * n_downsampling, col * n_downsampling);

    return eigen_image;
}

template <typename Scalar>
Eigen::Matrix<Scalar, -1, 1> Ros2EigenVector(
    const sensor_msgs::Image& ros_image,
    const size_t& n_downsampling = 1)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(ros_image)->image;

    size_t n_rows = cv_image.rows / n_downsampling;
    size_t n_cols = cv_image.cols / n_downsampling;

    Eigen::Matrix<Scalar, -1, 1> eigen_image(n_rows * n_cols, 1);
    for (size_t row = 0; row < n_rows; row++)
        for (size_t col = 0; col < n_cols; col++)
            eigen_image(row * n_cols + col) =
                cv_image.at<float>(row * n_downsampling, col * n_downsampling);

    return eigen_image;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> GetCameraMatrix(
    const std::string& camera_info_topic,
    ros::NodeHandle& node_handle,
    const Scalar& seconds)
{
    // TODO: Check if const pointer is valid before accessing memory
    sensor_msgs::CameraInfo::ConstPtr camera_info =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            camera_info_topic, node_handle, ros::Duration(seconds));

    Eigen::Matrix<Scalar, 3, 3> camera_matrix =
        Eigen::Matrix<Scalar, 3, 3>::Zero();

    if (!camera_info)
    {
        // if not topic was received within <seconds>
        ROS_WARN(
            "CameraInfo wasn't received within %f seconds. Returning default "
            "Zero message.",
            seconds);
        return camera_matrix;
    }
    ROS_INFO("Valid CameraInfo was received");

    for (size_t col = 0; col < 3; col++)
        for (size_t row = 0; row < 3; row++)
            camera_matrix(row, col) = camera_info->K[col + row * 3];

    return camera_matrix;
}

template <typename Scalar>
std::string GetCameraFrame(const std::string& camera_info_topic,
                           ros::NodeHandle& node_handle,
                           const Scalar& seconds)
{
    // TODO: Check if const pointer is valid before accessing memory
    sensor_msgs::CameraInfo::ConstPtr camera_info =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            camera_info_topic, node_handle, ros::Duration(seconds));
    if (!camera_info)
    {
        // if not topic was received within <seconds>
        ROS_WARN(
            "CameraInfo wasn't received within %f seconds. Returning default "
            "Zero message.",
            seconds);
        return "";
    }

    return camera_info->header.frame_id;
}

void PublishMarker(const Eigen::Matrix3f R,
                   const Eigen::Vector3f t,
                   std_msgs::Header header,
                   std::string object_model_path,
                   const ros::Publisher& pub,
                   int marker_id = 0,
                   float r = 0,
                   float g = 1,
                   float b = 0,
                   float a = 1.0,
                   std::string ns = "object");

void PublishMarker(const Eigen::Matrix4f H,
                   std_msgs::Header header,
                   std::string object_model_path,
                   const ros::Publisher& pub,
                   int marker_id = 0,
                   float r = 0,
                   float g = 1,
                   float b = 0,
                   float a = 1.0,
                   std::string ns = "object");

void PublishObjectState(const Eigen::Matrix4f H,
            std_msgs::Header header,
            std::string object_name,
            const ros::Publisher &pub);

void PublishObjectState(const Eigen::Matrix3f R,
            const Eigen::Vector3f t,
            std_msgs::Header header,
            std::string object_name,
            const ros::Publisher &pub);

void PublishPoints(const std_msgs::Header header,
                   const ros::Publisher& pub,
                   const std::vector<Eigen::Vector3f> points,
                   std::vector<float> colors = std::vector<float>(0),
                   const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
                   Eigen::Vector3f t = Eigen::Vector3f::Zero());
}