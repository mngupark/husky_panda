#include <iostream>
// ROS
#include "ros/ros.h"

// tf
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// messages
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
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
    void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void scan_merge();
    void get_transform();
    void set_transform();
    void merge_clouds();
    void convert_cloud_to_scan();

    // ROS param
    std::string front_scan_topic_, rear_scan_topic_, front_scan_mount_parent_link_, rear_scan_mount_parent_link_, front_scan_mount_link_, rear_scan_mount_link_, front_scan_link_, rear_scan_link_;
    std::string fake_scan_topic_, fake_scan_mount_frame_id_;
    std::string front_cloud_topic_, rear_cloud_topic_;
    
    // ROS Publisher & Subscriber
    ros::Publisher merged_scan_publisher_, merged_cloud_publisher_;
    ros::Publisher front_cloud_publisher_, rear_cloud_publisher_;
    ros::Subscriber front_scan_subscriber_, rear_scan_subscriber_;

    // tf
    tf2_ros::StaticTransformBroadcaster fake_scan_mount_broadcaster_, fake_scan_broadcaster_;
    // static tf::TransformBroadcaster fake_scan_mount_broadcaster, fake_scan_broadcaster;
    tf::TransformListener front_scan_mount_listener_, rear_scan_mount_listener_, front_scan_listener_, rear_scan_listener_;


    // laser scan & point cloud
    sensor_msgs::LaserScan front_scan_, rear_scan_;
    int seq_ = 0;
    int stamped_in_seq_ = 0;
    float deg_max_ = DEG2RAD(180.0);
    float deg_min_ = DEG2RAD(-180.0);
    bool use_inf_ = true;
    double inf_epsilon_ = 1.0;
    double tolerance_ = 0.01;
    unsigned int input_queue_size_ = 1;
    double max_height_ = std::numeric_limits<double>::max();
    double min_height_ = std::numeric_limits<double>::min();
    double range_min_ = 0.0;
    double range_max_ = std::numeric_limits<double>::max();
    Eigen::Matrix3d front_rotation_matrix_, rear_rotation_matrix_;
    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> message_filter_;

    sensor_msgs::PointCloud2 front_cloud_, rear_cloud_;
    sensor_msgs::PointCloud2 merged_cloud_;
    laser_geometry::LaserProjection front_scan_projector_, rear_scan_projector_;

    // tf
    tf::StampedTransform front_scan_mount_transform_, rear_scan_mount_transform_, front_scan_transform_, rear_scan_transform_;
    geometry_msgs::TransformStamped fake_scan_mount_transform_, fake_scan_transform_;
    // front scan transform
    geometry_msgs::Vector3 front_scan_mount_transform_origin_, rear_scan_mount_transform_origin_, fake_scan_mount_transform_origin_;
    geometry_msgs::Quaternion fake_scan_mount_transform_rotation_;
    // rear scan transform
    geometry_msgs::Vector3 front_scan_transform_origin_, rear_scan_transform_origin_, fake_scan_transform_origin_;
    geometry_msgs::Quaternion fake_scan_transform_rotation_;

    // pcl
    pcl::PointCloud<pcl::PointXYZ> front_pcl_cloud_, rear_pcl_cloud_;

public:
    HuskyPandaMobileScanMerger(/* args */);
    ~HuskyPandaMobileScanMerger();

    ros::NodeHandle nh_;
};

void HuskyPandaMobileScanMerger::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    front_scan_ = *scan;
    front_scan_projector_.transformLaserScanToPointCloud(front_scan_link_, front_scan_, front_cloud_, front_scan_listener_);
    pcl::fromROSMsg(front_cloud_, front_pcl_cloud_);
    // front_cloud_publisher_.publish(front_cloud_);
    return;
}

void HuskyPandaMobileScanMerger::rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    rear_scan_ = *scan;
    rear_scan_projector_.transformLaserScanToPointCloud(rear_scan_link_, rear_scan_, rear_cloud_, rear_scan_listener_);
    pcl::fromROSMsg(rear_cloud_, rear_pcl_cloud_);
    // rear_cloud_publisher_.publish(rear_cloud_);
    return;
}

void HuskyPandaMobileScanMerger::scan_merge() {
    while(nh_.ok()) {
        // set_transform(); // static tf is already generated in URDF   
        merge_clouds();
        // convert_cloud_to_scan();
        ros::spinOnce();
    }
}

void HuskyPandaMobileScanMerger::get_transform() {
    try
    {
        // Verify that TF knows how to transform from the received scan to the destination scan frame
        front_scan_mount_listener_.waitForTransform(front_scan_mount_parent_link_.c_str(), front_scan_mount_link_.c_str(), ros::Time::now(), ros::Duration(1));
        front_scan_listener_.waitForTransform(front_scan_mount_link_.c_str(), front_scan_link_.c_str(), ros::Time::now(), ros::Duration(1));
        rear_scan_mount_listener_.waitForTransform(rear_scan_mount_parent_link_.c_str(), rear_scan_mount_link_.c_str(), ros::Time::now(), ros::Duration(1));
        rear_scan_listener_.waitForTransform(rear_scan_mount_link_.c_str(), rear_scan_link_.c_str(), ros::Time::now(), ros::Duration(1));

        front_scan_mount_listener_.lookupTransform(front_scan_mount_parent_link_, front_scan_mount_link_,
                                                   ros::Time::now(), front_scan_mount_transform_);
        rear_scan_mount_listener_.lookupTransform(rear_scan_mount_parent_link_, rear_scan_mount_link_,
                                                  ros::Time::now(), rear_scan_mount_transform_);
        front_scan_listener_.lookupTransform(front_scan_mount_link_, front_scan_link_,
                                             ros::Time::now(), front_scan_transform_);
        rear_scan_listener_.lookupTransform(rear_scan_mount_link_, rear_scan_link_,
                                            ros::Time::now(), rear_scan_transform_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    /// @todo get laser scan from two LiDARs and fusion them into a unique laser scan.
    ///       Broadcast this scan at the middle pose of each scan.
    // // try
    // // {
    // //     front_scan_mount_listener_.lookupTransform(front_scan_mount_parent_link_, front_scan_mount_link_,
    // //                              ros::Time::now(), front_scan_mount_transform_);
    // // }
    // // catch (tf::TransformException ex)
    // // {
    // //     ROS_ERROR("%s", ex.what());
    // //     ros::Duration(1.0).sleep();
    // // }
    //
    // // try
    // // {
    // //     rear_scan_mount_listener_.lookupTransform(rear_scan_mount_parent_link_, rear_scan_mount_link_,
    // //                              ros::Time::now(), rear_scan_mount_transform_);
    // // }
    // // catch (tf::TransformException ex)
    // // {
    // //     ROS_ERROR("%s", ex.what());
    // //     ros::Duration(1.0).sleep();
    // // }
    //
    // // try
    // // {
    // //     front_scan_listener_.lookupTransform(front_scan_mount_link_, front_scan_link_,
    // //                              ros::Time::now(), front_scan_transform_);
    // // }
    // // catch (tf::TransformException ex)
    // // {
    // //     ROS_ERROR("%s", ex.what());
    // //     ros::Duration(1.0).sleep();
    // // }
    //
    // // try
    // // {
    // //     rear_scan_listener_.lookupTransform(rear_scan_mount_link_, rear_scan_link_,
    // //                              ros::Time::now(), rear_scan_transform_);
    // // }
    // // catch (tf::TransformException ex)
    // // {
    // //     ROS_ERROR("%s", ex.what());
    // //     ros::Duration(1.0).sleep();
    // // }

    // tf::StampedTransform fake_scan_mount_transform, fake_scan_transform;
    // fake_scan_mount_transform.frame_id_ = fake_scan_frame_id;
    // fake_scan_mount_transform.child_frame_id_ = "fake_scan_mount_link";
    // fake_scan_mount_transform.setOrigin((front_scan_mount_transform.getOrigin() + rear_scan_mount_transform.getOrigin()) / 2.0);
    // fake_scan_mount_transform.setRotation(tf::Quaternion::getIdentity());
    // fake_scan_mount_broadcaster.sendTransform(fake_scan_mount_transform);
    //
    // fake_scan_transform.frame_id_ = "fake_scan_mount_link";
    // fake_scan_transform.child_frame_id_ = "fake_scan_link";
    // fake_scan_transform.setOrigin((front_scan_transform.getOrigin() + rear_scan_transform.getOrigin()) / 2.0);
    // fake_scan_transform.setRotation(tf::Quaternion::getIdentity());
    // fake_scan_broadcaster.sendTransform(fake_scan_transform);


    fake_scan_mount_transform_.header.frame_id = fake_scan_mount_frame_id_;
    fake_scan_mount_transform_.child_frame_id = "fake_scan_mount_link";

    tf::vector3TFToMsg(front_scan_mount_transform_.getOrigin(), front_scan_mount_transform_origin_);
    tf::vector3TFToMsg(rear_scan_mount_transform_.getOrigin(), rear_scan_mount_transform_origin_);
    fake_scan_mount_transform_origin_.x = (front_scan_mount_transform_origin_.x + rear_scan_mount_transform_origin_.x) / 2.0;
    fake_scan_mount_transform_origin_.y = (front_scan_mount_transform_origin_.y + rear_scan_mount_transform_origin_.y) / 2.0;
    fake_scan_mount_transform_origin_.z = (front_scan_mount_transform_origin_.z + rear_scan_mount_transform_origin_.z) / 2.0;
    fake_scan_mount_transform_.transform.translation = fake_scan_mount_transform_origin_;
    fake_scan_mount_transform_rotation_.w = 1.0;
    fake_scan_mount_transform_rotation_.x = 0.0;
    fake_scan_mount_transform_rotation_.y = 0.0;
    fake_scan_mount_transform_rotation_.z = 0.0;
    fake_scan_mount_transform_.transform.rotation = fake_scan_mount_transform_rotation_;
    // fake_scan_mount_broadcaster_.sendTransform(fake_scan_mount_transform_);

    fake_scan_transform_.header.frame_id = "fake_scan_mount_link";
    fake_scan_transform_.child_frame_id = "fake_scan_link";
    tf::vector3TFToMsg(front_scan_transform_.getOrigin(), front_scan_transform_origin_);
    tf::vector3TFToMsg(rear_scan_transform_.getOrigin(), rear_scan_transform_origin_);
    fake_scan_transform_origin_.x = (front_scan_transform_origin_.x + rear_scan_transform_origin_.x) / 2.0;
    fake_scan_transform_origin_.y = (front_scan_transform_origin_.y + rear_scan_transform_origin_.y) / 2.0;
    fake_scan_transform_origin_.z = (front_scan_transform_origin_.z + rear_scan_transform_origin_.z) / 2.0;
    fake_scan_transform_.transform.translation = fake_scan_transform_origin_;
    fake_scan_transform_rotation_.w = 1.0;
    fake_scan_transform_rotation_.x = 0.0;
    fake_scan_transform_rotation_.y = 0.0;
    fake_scan_transform_rotation_.z = 0.0;
    fake_scan_transform_.transform.rotation = fake_scan_transform_rotation_;
    // fake_scan_broadcaster_.sendTransform(fake_scan_transform_);

    return;
}

void HuskyPandaMobileScanMerger::set_transform() {
    fake_scan_mount_broadcaster_.sendTransform(fake_scan_mount_transform_);
    fake_scan_broadcaster_.sendTransform(fake_scan_transform_);
}

void HuskyPandaMobileScanMerger::merge_clouds() {
    pcl::PointCloud<pcl::PointXYZ> transformed_front_pcl, transformed_rear_pcl;
    tf::TransformListener front_to_fake_listener, rear_to_fake_listener;
    tf::StampedTransform fake_to_front_transform, rear_to_fake_transform;
    try
    {
        // Verify that TF knows how to transform from the received scan to the destination scan frame
        front_to_fake_listener.waitForTransform("fake_scan_link", front_scan_link_.c_str() , ros::Time::now(), ros::Duration(1));
        rear_to_fake_listener.waitForTransform("fake_scan_link", rear_scan_link_.c_str(), ros::Time::now(), ros::Duration(1));

        front_to_fake_listener.lookupTransform("fake_scan_link", front_scan_link_.c_str(),
                                                   ros::Time::now(), fake_to_front_transform);
        rear_to_fake_listener.lookupTransform("fake_scan_link", rear_scan_link_.c_str(),
                                                   ros::Time::now(), rear_to_fake_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    Eigen::Vector3d fake_to_front_translation, fake_to_rear_translation;
    Eigen::Matrix3d fake_to_front_rotation, fake_to_rear_rotation;
    Eigen::Quaterniond fake_to_front_quaternion, fake_to_rear_quaternion;
    tf::vectorTFToEigen(fake_to_front_transform.getOrigin(), fake_to_front_translation);
    tf::vectorTFToEigen(rear_to_fake_transform.getOrigin(), fake_to_rear_translation);
    tf::quaternionTFToEigen(fake_to_front_transform.getRotation(), fake_to_front_quaternion);
    tf::quaternionTFToEigen(rear_to_fake_transform.getRotation(), fake_to_rear_quaternion);
    fake_to_front_rotation = fake_to_front_quaternion.normalized().toRotationMatrix();
    fake_to_rear_rotation = fake_to_rear_quaternion.normalized().toRotationMatrix();
    Eigen::Matrix4d fake_to_front_matrix, fake_to_rear_matrix;
    fake_to_front_matrix.setZero();
    fake_to_front_matrix.block<3, 3>(0, 0) = fake_to_front_rotation;
    fake_to_front_matrix.block<3, 1>(0, 3) = fake_to_front_translation;
    fake_to_front_matrix(3, 3) = 1.0;
    fake_to_rear_matrix.setZero();
    fake_to_rear_matrix.block<3, 3>(0, 0) = fake_to_rear_rotation;
    fake_to_rear_matrix.block<3, 1>(0, 3) = fake_to_rear_translation;
    fake_to_rear_matrix(3, 3) = 1.0;
    
    pcl::transformPointCloud(front_pcl_cloud_, transformed_front_pcl, fake_to_front_matrix, true);
    pcl::transformPointCloud(rear_pcl_cloud_, transformed_rear_pcl, fake_to_rear_matrix, true);
    // transformed_front_pcl.header.frame_id = "fake_scan_link";
    // transformed_rear_pcl.header.frame_id = "fake_scan_link";

    // Merge two different point clouds when there are duplicated points
    /// @TODO: Filtering the point clouds into one point cloud using KDTree(K-Dimensional Tree).
    pcl::PointCloud<pcl::PointXYZ> merged_pcl, filtered_pcl;
    merged_pcl.header.frame_id = "fake_scan_link";
    pcl_conversions::toPCL(ros::Time::now(), merged_pcl.header.stamp);
    merged_pcl += transformed_front_pcl;
    merged_pcl += transformed_rear_pcl;

    // Publish ROS topic after converting clouds from pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 temp_front_cloud, temp_rear_cloud;
    pcl::toROSMsg(transformed_front_pcl, temp_front_cloud);
    pcl::toROSMsg(transformed_rear_pcl, temp_rear_cloud);
    pcl::toROSMsg(merged_pcl, merged_cloud_);
    temp_front_cloud.header.frame_id = "fake_scan_link";
    temp_rear_cloud.header.frame_id = "fake_scan_link";
    // // // // ROS_INFO("%s %s", temp_front_cloud.header.frame_id.c_str(), temp_rear_cloud.header.frame_id.c_str());
    front_cloud_publisher_.publish(temp_front_cloud);
    rear_cloud_publisher_.publish(temp_rear_cloud);
    merged_cloud_publisher_.publish(merged_cloud_);
}

void HuskyPandaMobileScanMerger::convert_cloud_to_scan() {
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = merged_cloud_.header;
  output.header.frame_id = "fake_scan_link";

  output.angle_min = deg_max_;
  output.angle_max = deg_min_;
  output.angle_increment = M_PI / 360.0;
  output.time_increment = 0.0;
  output.scan_time = std::min(front_scan_.scan_time, rear_scan_.scan_time);
  output.range_min = range_min_;//std::max(front_scan_.range_min, rear_scan_.range_min);
  output.range_max = range_max_;//std::min(front_scan_.range_max, rear_scan_.range_max);

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
//   if (!(output.header.frame_id == merged_cloud_.header.frame_id))
//   {
//     try
//     {
//       cloud.reset(new sensor_msgs::PointCloud2);
//       tf2_->transform(merged_cloud_, *cloud, "fake_scan_link", ros::Duration(tolerance_));
//       cloud_out = cloud;
//     }
//     catch (tf2::TransformException& ex)
//     {
//       ROS_ERROR("%s", ex.what());
//       return;
//     }
//   }
//   else
//   {
    cloud_out = boost::make_shared<const sensor_msgs::PointCloud2>(merged_cloud_);
//   }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  merged_scan_publisher_.publish(output);
}

HuskyPandaMobileScanMerger::HuskyPandaMobileScanMerger(/* args */)
{
    ros::NodeHandle nh("~");

    nh.param<std::string>("front_scan_topic", front_scan_topic_, "/front_scan");
    nh.param<std::string>("front_cloud_topic", front_cloud_topic_, "/front_cloud");
    nh.param<std::string>("front_scan_mount_parent_link", front_scan_mount_parent_link_, "top_plate_front_link");
    nh.param<std::string>("front_scan_mount_link", front_scan_mount_link_, "laser_mount_link");
    nh.param<std::string>("front_scan_link", front_scan_link_, "laser");

    nh.param<std::string>("rear_scan_topic", rear_scan_topic_, "/rear_scan");
    nh.param<std::string>("rear_cloud_topic", rear_cloud_topic_, "/rear_cloud");
    nh.param<std::string>("rear_scan_mount_parent_link", rear_scan_mount_parent_link_, "top_plate_rear_link");
    nh.param<std::string>("rear_scan_mount_link", rear_scan_mount_link_, "hokuyo_laser_mount");
    nh.param<std::string>("rear_scan_link", rear_scan_link_, "hokuyo_laser");
    
    nh.param<std::string>("fake_scan_topic", fake_scan_topic_, "/fake_scan");
    nh.param<std::string>("fake_scan_mount_frame_id", fake_scan_mount_frame_id_, "top_plate_link");

    // ROS Publisher & Subscriber
    front_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(front_scan_topic_, 50, &HuskyPandaMobileScanMerger::front_scan_callback, this);
    rear_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(rear_scan_topic_, 50, &HuskyPandaMobileScanMerger::rear_scan_callback, this);
    merged_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 100, false);
    front_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> (front_cloud_topic_, 100, false);
    rear_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> (rear_cloud_topic_, 100, false);
    merged_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> ("/merged_cloud", 100, false);

    // listen for the tf transform
    // get_transform();

    // calculate the rotation matrix for skrewed scan
    front_rotation_matrix_.setZero();
    rear_rotation_matrix_.setZero();
    double front_theta = DEG2RAD(45.0);
    double rear_theta = DEG2RAD(-135.0);
    front_rotation_matrix_ <<   cos(front_theta), -sin(front_theta),  0,
                                sin(front_theta),  cos(front_theta),  0,
                                0               ,  0               ,  1;
    rear_rotation_matrix_ <<    cos(rear_theta), -sin(rear_theta),  0,
                                sin(rear_theta),  cos(rear_theta),  0,
                                0              ,  0              ,  1;

    // Start merging
    scan_merge();
}

HuskyPandaMobileScanMerger::~HuskyPandaMobileScanMerger()
{
}

// scan tf 5.1cm from mount link
// tf sick: laser_mount_link -> laser
// tf hokuyo: hokuyo_laser_mount -> hokuyo_laser
int main(int argc, char ** argv) {
    // initialize
    ros::init(argc, argv, "husky_panda_mobile_scan_node");
    HuskyPandaMobileScanMerger husky_panda_mobile_scan_merger;
    return 0;
}