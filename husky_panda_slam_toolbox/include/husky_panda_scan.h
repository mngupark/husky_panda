#pragma once
#ifndef __HUSKY_PANDA_SCAN_H__
#define __HUSKY_PANDA_SCAN_H__
#include <iostream>
#include <memory>
// ROS
#include "ros/ros.h"

// tf
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// messages
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"

// pcl
#include "laser_geometry/laser_geometry.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

class HuskyPandaMobileScanMerger
{
private:
  void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void scan_merge();
  void get_static_transform();
  void merge_clouds();

  // ROS param
  std::string front_scan_topic_, rear_scan_topic_, fake_scan_topic_;   // topic name of the sensor_msgs/LaserScan type message
  std::string front_cloud_topic_, rear_cloud_topic_, fake_cloud_topic; // topic name of the sensor_msgs/PointCloud2 type message
  std::string front_scan_frame_id_, rear_scan_frame_id_;               // tf frame id of the front and rear LiDAR
  std::string fake_scan_mount_frame_id_, fake_scan_frame_id_;          // tf frame id of the fake scan

  // ROS Publisher & Subscriber
  ros::Subscriber front_scan_subscriber_, rear_scan_subscriber_; // subscriber of the sensor_msgs/LaserScan type message
  ros::Publisher front_cloud_publisher_, rear_cloud_publisher_;  // publisher of the sensor_msgs/PointCloud2 type message which is converted from subscribed laser scan
  ros::Publisher fake_scan_publisher, fake_cloud_publisher;      // publisher of the generated fake scan and cloud ROS messages

  // ROS messages
  // laser scan & point cloud
  boost::shared_ptr<sensor_msgs::LaserScan> front_scan_, rear_scan_;                         // boost shared pointer of the topic sensor_msgs/Laserscan which point to the subscribed laser scan
  boost::shared_ptr<sensor_msgs::PointCloud2> front_cloud_, rear_cloud_, fake_cloud_;        // boost shared pointer of the topic sensor_msgs/PointCloud2
  laser_geometry::LaserProjection front_scan_projector_, rear_scan_projector_; // project the laser scan to point cloud

  // tf
  tf2_ros::StaticTransformBroadcaster fake_scan_mount_broadcaster_, fake_scan_broadcaster_; // tf static transform publisher of the fake scan
  tf::TransformListener front_scan_listener_, rear_scan_listener_;                          // tf transform listener which listen to the transformation from scan to cloud
  tf::TransformListener fake_to_front_listener_, fake_to_rear_llistener_;                   // tf transform listener which listen to the transformation from front(or rear) LiDAR frame to fake frame
  tf::StampedTransform fake_to_front_transform_, fake_to_rear_transform_;                   // tf stamped transform which contains the transformation from front(or rear) LiDAR frame

  // pcl
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> front_pcl_cloud_, rear_pcl_cloud_;        // boost shared pointer of the pcl cloud which is converted from ROS message (sensor_msgs/PointCloud2)
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed_front_pcl_cloud_, transformed_rear_pcl_cloud_; // boost shared pointer of the pcl cloud which is transformed its own frame from front(or rear) LiDAR frame to fake LiDAR frame

  // Eigen
  Eigen::Matrix3d front_rotation_matrix_, rear_rotation_matrix_;        // rotation matrix of each LiDAR frame
  Eigen::Vector3d fake_to_front_translation, fake_to_rear_translation;  // translation vector(3x1) of homogenous matrix from fake LiDAR frame to front(or rear) LiDAR frame
  Eigen::Matrix3d fake_to_front_rotation, fake_to_rear_rotation;        // rotation matrix(3x3) of homogenous matrix from fake LiDAR frame to front(or rear) LiDAR frame
  Eigen::Matrix4d fake_to_front_matrix, fake_to_rear_matrix;            // homogeneous matrix(4x4) of transformation from fake LiDAR frame to front(or rear) LiDAR frame
  Eigen::Quaterniond fake_to_front_quaternion, fake_to_rear_quaternion; // quaternion vector(4x1) which is converted from tf::StampedTransform to Eigen

public:
  HuskyPandaMobileScanMerger();
  ~HuskyPandaMobileScanMerger();

  ros::NodeHandle nh_;
};

#endif