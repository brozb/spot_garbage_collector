#ifndef TRAVERSABILITY_BUILDER_H
#define TRAVERSABILITY_BUILDER_H

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
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer_p;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener_p;

    bool* grid_received;

    ros::Publisher pcl_pub;
    ros::Subscriber sub;
    ros::Timer timer;

    sensor_msgs::PointCloud2Ptr trav;
    sensor_msgs::PointCloud2Ptr last_grid;

    boost::mutex* cloud_lock_p;

    Filter(ros::NodeHandle *nh);
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void callback_tim(const ros::TimerEvent&);
};

#endif
