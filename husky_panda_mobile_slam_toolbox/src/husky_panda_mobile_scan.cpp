#include <iostream>
// ROS
#include "ros/ros.h"

// tf
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// messages
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"

// pcl
#include "laser_geometry/laser_geometry.h"

class HuskyPandaMobileScanMerger
{
private:
    float RAD2DEG(float rad) {
        return 180.0 * rad / M_PI;
    };
    float DEG2RAD(float deg) {
        return M_PI * deg / 180.0;
    };
    
    void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void scan_merge();
    void get_and_set_transform();
    void convert_scan_to_cloud();
    void merge_clouds();
    void convert_cloud_to_scan();

    // ROS param
    std::string front_scan_topic_, rear_scan_topic_, front_scan_mount_parent_link_, rear_scan_mount_parent_link_, front_scan_mount_link_, rear_scan_mount_link_, front_scan_link_, rear_scan_link_;
    std::string fake_scan_topic_, fake_scan_mount_frame_id_;
    std::string front_cloud_topic_, rear_cloud_topic_;
    
    // ROS Publisher & Subscriber
    ros::Publisher laser_scan_publisher_;
    ros::Publisher front_cloud_publisher_, rear_cloud_publisher_;
    ros::Subscriber front_scan_subscriber_, rear_scan_subscriber_;

    // tf
    tf2_ros::StaticTransformBroadcaster fake_scan_mount_broadcaster_, fake_scan_broadcaster_;
    // static tf::TransformBroadcaster fake_scan_mount_broadcaster, fake_scan_broadcaster;
    tf::TransformListener front_scan_mount_listener_, rear_scan_mount_listener_, front_scan_listener_, rear_scan_listener_;


    // laser scan & point cloud
    sensor_msgs::LaserScan front_scan_, rear_scan_;
    int seq_ = 0;
    float deg_max_ = DEG2RAD(180.0);
    float deg_min_ = -DEG2RAD(90.0);

    sensor_msgs::PointCloud2 front_cloud_, rear_cloud_;
    laser_geometry::LaserProjection front_scan_projector_, rear_scan_projector_;

    // tf
    tf::StampedTransform front_scan_mount_transform_, rear_scan_mount_transform_, front_scan_transform_, rear_scan_transform_;
    geometry_msgs::TransformStamped fake_scan_mount_transform, fake_scan_transform;
    // front scan transform
    geometry_msgs::Vector3 front_scan_mount_transform_origin, rear_scan_mount_transform_origin, fake_scan_mount_transform_origin;
    geometry_msgs::Quaternion fake_scan_mount_transform_rotation;
    // rear scan transform
    geometry_msgs::Vector3 front_scan_transform_origin, rear_scan_transform_origin, fake_scan_transform_origin;
    geometry_msgs::Quaternion fake_scan_transform_rotation;

public:
    HuskyPandaMobileScanMerger(/* args */);
    ~HuskyPandaMobileScanMerger();

    ros::NodeHandle nh_;
};

void HuskyPandaMobileScanMerger::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    front_scan_ = *scan;
    front_scan_projector_.transformLaserScanToPointCloud(front_scan_link_, front_scan_, front_cloud_, front_scan_listener_);
    front_cloud_publisher_.publish(front_cloud_);
    return;
}

void HuskyPandaMobileScanMerger::rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    rear_scan_ = *scan;
    return;
}

void HuskyPandaMobileScanMerger::scan_merge() {
    while(nh_.ok()) {
        get_and_set_transform();
        convert_scan_to_cloud();
        merge_clouds();
        convert_cloud_to_scan();
        ros::spinOnce();
    }
}

void HuskyPandaMobileScanMerger::get_and_set_transform() {
        /// @todo get laser scan from two LiDARs and fusion them into a unique laser scan.
        ///       Broadcast this scan at the middle pose of each scan.
        try
        {
            front_scan_mount_listener_.lookupTransform(front_scan_mount_parent_link_, front_scan_mount_link_,
                                     ros::Time::now(), front_scan_mount_transform_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        try
        {
            rear_scan_mount_listener_.lookupTransform(rear_scan_mount_parent_link_, rear_scan_mount_link_,
                                     ros::Time::now(), rear_scan_mount_transform_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        try
        {
            front_scan_listener_.lookupTransform(front_scan_mount_link_, front_scan_link_,
                                     ros::Time::now(), front_scan_transform_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        try
        {
            rear_scan_listener_.lookupTransform(rear_scan_mount_link_, rear_scan_link_,
                                     ros::Time::now(), rear_scan_transform_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

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


        fake_scan_mount_transform.header.frame_id = fake_scan_mount_frame_id_;
        fake_scan_mount_transform.child_frame_id = "fake_scan_mount_link";

        tf::vector3TFToMsg(front_scan_mount_transform_.getOrigin(), front_scan_mount_transform_origin);
        tf::vector3TFToMsg(rear_scan_mount_transform_.getOrigin(), rear_scan_mount_transform_origin);
        fake_scan_mount_transform_origin.x = (front_scan_mount_transform_origin.x + rear_scan_mount_transform_origin.x) / 2.0;
        fake_scan_mount_transform_origin.y = (front_scan_mount_transform_origin.y + rear_scan_mount_transform_origin.y) / 2.0;
        fake_scan_mount_transform_origin.z = (front_scan_mount_transform_origin.z + rear_scan_mount_transform_origin.z) / 2.0;
        fake_scan_mount_transform.transform.translation = fake_scan_mount_transform_origin;
        fake_scan_mount_transform_rotation.w = 1.0;
        fake_scan_mount_transform_rotation.x = 0.0;
        fake_scan_mount_transform_rotation.y = 0.0;
        fake_scan_mount_transform_rotation.z = 0.0;
        fake_scan_mount_transform.transform.rotation = fake_scan_mount_transform_rotation;
        fake_scan_mount_broadcaster_.sendTransform(fake_scan_mount_transform);

        fake_scan_transform.header.frame_id = "fake_scan_mount_link";
        fake_scan_transform.child_frame_id = "fake_scan_link";
        tf::vector3TFToMsg(front_scan_transform_.getOrigin(), front_scan_transform_origin);
        tf::vector3TFToMsg(rear_scan_transform_.getOrigin(), rear_scan_transform_origin);
        fake_scan_transform_origin.x = (front_scan_transform_origin.x + rear_scan_transform_origin.x) / 2.0;
        fake_scan_transform_origin.y = (front_scan_transform_origin.y + rear_scan_transform_origin.y) / 2.0;
        fake_scan_transform_origin.z = (front_scan_transform_origin.z + rear_scan_transform_origin.z) / 2.0;
        fake_scan_transform.transform.translation = fake_scan_transform_origin;
        fake_scan_transform_rotation.w = 1.0;
        fake_scan_transform_rotation.x = 0.0;
        fake_scan_transform_rotation.y = 0.0;
        fake_scan_transform_rotation.z = 0.0;
        fake_scan_transform.transform.rotation = fake_scan_transform_rotation;
        fake_scan_broadcaster_.sendTransform(fake_scan_transform);

        return;
}

void HuskyPandaMobileScanMerger::convert_scan_to_cloud() {

}

void HuskyPandaMobileScanMerger::merge_clouds() {

}

void HuskyPandaMobileScanMerger::convert_cloud_to_scan() {
    //     sensor_msgs::LaserScan scan;
    //     scan.header.frame_id = fake_scan_frame_id_;
    //     scan.header.stamp = ros::Time::now();
    //     scan.header.seq = seq++;
    //     scan.angle_max = deg_max;
    //     scan.angle_min = deg_min;
    //     scan.angle_increment = (front_scan.angle_increment + rear_scan.angle_increment) / 2.0;
    //     //front_scan.intensities.insert(front_scan.intensities.end(), rear_scan.intensities.begin(), rear_scan.intensities.end());
    //     //scan.intensities = front_scan.intensities;
    //     scan.range_max = std::min(front_scan.range_max, rear_scan.range_max);
    //     scan.range_min = std::max(front_scan.range_min, rear_scan.range_min);
    //     //front_scan.ranges.insert(front_scan.ranges.end(), rear_scan.ranges.begin(), rear_scan.ranges.end());
    //     scan.ranges = front_scan.ranges;
    //     scan.scan_time = std::min(front_scan.scan_time, rear_scan.scan_time);
    //     scan.time_increment = std::min(front_scan.time_increment, rear_scan.time_increment);
    //     laser_scan_publisher.publish(scan);
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
    laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 1000);
    front_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> (front_cloud_topic_, 100, false);
    rear_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> (rear_cloud_topic_, 100, false);

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