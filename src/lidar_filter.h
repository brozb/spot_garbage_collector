#ifndef CONVOYING_LIDAR_FILTER_H
#define CONVOYING_LIDAR_FILTER_H

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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
const float bad_point = std::numeric_limits<float>::quiet_NaN();

class Filter {
public:
    ros::Publisher pcl_pub;
    ros::Subscriber sub;
    Filter(ros::NodeHandle *nh);
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

#endif
