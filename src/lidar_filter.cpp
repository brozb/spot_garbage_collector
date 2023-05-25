#include "lidar_filter.h"
using namespace std;

Filter::Filter(ros::NodeHandle *nh) {
    pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("/pcl_filtered", 1);
    ros::Duration(1).sleep();
    ROS_INFO("filtering started");
    sub = nh->subscribe("/points", 1, &Filter::callback, this);
}

void Filter::callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.05, 0.05, 0.02);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pcl_pub.publish (output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_filter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Filter pcl_filter = Filter(&nh);
    ros::spin();
}
