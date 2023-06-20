#ifndef LIDAR_FILTER_H
#define LIDAR_FILTER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <math.h>
#include <string>
#include <vector>
#include <mutex>

#include <pcl/common/io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <cstdio>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
const float bad_point = std::numeric_limits<float>::quiet_NaN();

class Filter {
public:
    tf2_ros::Buffer* tfBuffer_p;
    tf2_ros::TransformListener* tfListener_p;

    bool* fl_received;
    bool* fr_received;
    bool* l_received;
    bool* r_received;
    bool* b_received;

    ros::Publisher pcl_pub;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Timer timer;

    sensor_msgs::PointCloud2Ptr cloud_fl;
    sensor_msgs::PointCloud2Ptr cloud_fr;
    sensor_msgs::PointCloud2Ptr cloud_l;
    sensor_msgs::PointCloud2Ptr cloud_r;
    sensor_msgs::PointCloud2Ptr cloud_b;

    boost::mutex* cloud_lock_p;

    Filter(ros::NodeHandle *nh);
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg, sensor_msgs::PointCloud2Ptr& cloud_pointer, bool*& status, const char* name);
    void callback_tim(const ros::TimerEvent&);
};

#endif
