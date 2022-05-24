#include "husky_panda_scan.h"

void HuskyPandaMobileScanMerger::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  *front_scan_ = *scan;
  front_scan_projector_.transformLaserScanToPointCloud(front_scan_frame_id_, *front_scan_, *front_cloud_, front_scan_listener_);
  pcl::fromROSMsg(*front_cloud_, *front_pcl_cloud_);
  return;
}

void HuskyPandaMobileScanMerger::rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  *rear_scan_ = *scan;
  rear_scan_projector_.transformLaserScanToPointCloud(rear_scan_frame_id_, *rear_scan_, *rear_cloud_, rear_scan_listener_);
  pcl::fromROSMsg(*rear_cloud_, *rear_pcl_cloud_);
  return;
}

void HuskyPandaMobileScanMerger::scan_merge()
{
  while (nh_.ok())
  {
    // get static transformations about front and rear scan
    get_static_transform();
    merge_clouds();
    ros::spinOnce();
  }
}

void HuskyPandaMobileScanMerger::get_static_transform()
{
  // ROS_INFO("%s",front_scan_frame_id_.c_str());
  try
  {
    // Verify that TF knows how to transform from the received scan to the destination scan frame
    fake_to_front_listener_.waitForTransform(fake_scan_frame_id_, front_scan_frame_id_.c_str(), ros::Time::now(), ros::Duration(1));
    fake_to_rear_llistener_.waitForTransform(fake_scan_frame_id_, rear_scan_frame_id_.c_str(), ros::Time::now(), ros::Duration(1));

    fake_to_front_listener_.lookupTransform(fake_scan_frame_id_, front_scan_frame_id_.c_str(),
                                            ros::Time::now(), fake_to_front_transform_);
    fake_to_rear_llistener_.lookupTransform(fake_scan_frame_id_, rear_scan_frame_id_.c_str(),
                                            ros::Time::now(), fake_to_rear_transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  tf::vectorTFToEigen(fake_to_front_transform_.getOrigin(), fake_to_front_translation);
  tf::vectorTFToEigen(fake_to_rear_transform_.getOrigin(), fake_to_rear_translation);
  tf::quaternionTFToEigen(fake_to_front_transform_.getRotation(), fake_to_front_quaternion);
  tf::quaternionTFToEigen(fake_to_rear_transform_.getRotation(), fake_to_rear_quaternion);
  fake_to_front_rotation = fake_to_front_quaternion.normalized().toRotationMatrix();
  fake_to_rear_rotation = fake_to_rear_quaternion.normalized().toRotationMatrix();
  fake_to_front_matrix.setZero();
  fake_to_front_matrix.block<3, 3>(0, 0) = fake_to_front_rotation; // * front_rotation_matrix_; // Rotate the front LiDAR frame about 5 degrees
  fake_to_front_matrix.block<3, 1>(0, 3) = fake_to_front_translation;
  fake_to_front_matrix(3, 3) = 1.0;
  fake_to_rear_matrix.setZero();
  fake_to_rear_matrix.block<3, 3>(0, 0) = fake_to_rear_rotation;
  fake_to_rear_matrix.block<3, 1>(0, 3) = fake_to_rear_translation;
  fake_to_rear_matrix(3, 3) = 1.0;
}

void HuskyPandaMobileScanMerger::merge_clouds() // Merge two different point clouds when there are duplicated points
{
  // transform point cloud from subscribed sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::transformPointCloud(*front_pcl_cloud_, *transformed_front_pcl_cloud_, fake_to_front_matrix, true);
  pcl::transformPointCloud(*rear_pcl_cloud_, *transformed_rear_pcl_cloud_, fake_to_rear_matrix, true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // boost shared pointer of the pcl cloud which has merged cloud of front and rear clouds
  merged_pcl_cloud->header.frame_id = fake_scan_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), merged_pcl_cloud->header.stamp);
  *merged_pcl_cloud += *transformed_front_pcl_cloud_;
  *merged_pcl_cloud += *transformed_rear_pcl_cloud_;

  // Publish ROS topic after converting clouds from pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 temp_front_cloud, temp_rear_cloud;
  pcl::toROSMsg(*transformed_front_pcl_cloud_, temp_front_cloud);
  pcl::toROSMsg(*transformed_rear_pcl_cloud_, temp_rear_cloud);
  pcl::toROSMsg(*merged_pcl_cloud, *fake_cloud_);
  temp_front_cloud.header.frame_id = fake_scan_frame_id_;
  temp_rear_cloud.header.frame_id = fake_scan_frame_id_;
  front_cloud_publisher_.publish(temp_front_cloud);
  rear_cloud_publisher_.publish(temp_rear_cloud);
  fake_cloud_publisher.publish(fake_cloud_);
}

HuskyPandaMobileScanMerger::HuskyPandaMobileScanMerger()
{
  ros::NodeHandle nh("~");

  nh.param<std::string>("front_scan_topic", front_scan_topic_, "/front_scan");
  nh.param<std::string>("front_cloud_topic", front_cloud_topic_, "/front_cloud");
  nh.param<std::string>("front_scan_frame_id", front_scan_frame_id_, "front_laser");

  nh.param<std::string>("rear_scan_topic", rear_scan_topic_, "/rear_scan");
  nh.param<std::string>("rear_cloud_topic", rear_cloud_topic_, "/rear_cloud");
  nh.param<std::string>("rear_scan_frame_id", rear_scan_frame_id_, "rear_laser");

  nh.param<std::string>("fake_scan_topic", fake_scan_topic_, "/fake_scan");
  nh.param<std::string>("fake_cloud_topic", fake_cloud_topic, "/fake_cloud");
  nh.param<std::string>("fake_scan_mount_frame_id", fake_scan_mount_frame_id_, "top_plate_link");
  nh.param<std::string>("fake_scan_frame_id", fake_scan_frame_id_, "fake_laser");

  // ROS Publisher & Subscriber
  front_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(front_scan_topic_, 50, &HuskyPandaMobileScanMerger::front_scan_callback, this);
  rear_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(rear_scan_topic_, 50, &HuskyPandaMobileScanMerger::rear_scan_callback, this);
  fake_scan_publisher = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 100, false);
  front_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(front_cloud_topic_, 100, false);
  rear_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(rear_cloud_topic_, 100, false);
  fake_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>(fake_cloud_topic, 100, false);

  // initialize boost shared pointers
  front_scan_.reset(new sensor_msgs::LaserScan);
  rear_scan_.reset(new sensor_msgs::LaserScan);
  front_cloud_.reset(new sensor_msgs::PointCloud2);
  rear_cloud_.reset(new sensor_msgs::PointCloud2);
  fake_cloud_.reset(new sensor_msgs::PointCloud2);
  front_pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  rear_pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_front_pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_rear_pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // initialization of all variables & classes
  front_rotation_matrix_.setZero();
  rear_rotation_matrix_.setZero();
  fake_to_front_translation.setZero();
  fake_to_rear_translation.setZero();
  fake_to_front_rotation.setZero();
  fake_to_rear_rotation.setZero();
  fake_to_front_matrix.setZero();
  fake_to_rear_matrix.setZero();
  fake_to_front_quaternion.vec().setZero();
  fake_to_front_quaternion.w() = 0.0;
  fake_to_rear_quaternion.vec().setZero();
  fake_to_rear_quaternion.w() = 0.0;

  // calculate the rotation matrix for each scans if the frame is skrewed
  front_rotation_matrix_.setZero();
  rear_rotation_matrix_.setZero();
  double front_theta = DEG2RAD(5.0);
  double rear_theta = DEG2RAD(0.0);
  front_rotation_matrix_ << cos(front_theta), -sin(front_theta), 0,
      sin(front_theta), cos(front_theta), 0,
      0, 0, 1;
  rear_rotation_matrix_ << cos(rear_theta), -sin(rear_theta), 0,
      sin(rear_theta), cos(rear_theta), 0,
      0, 0, 1;

  // Start merging
  scan_merge();
}

HuskyPandaMobileScanMerger::~HuskyPandaMobileScanMerger()
{
}