#include "lidar_filter.h"
using namespace std;

#if PCL_VERSION_COMPARE(<, 1, 10, 0)
#error "PCL of version at least 1.10 is required"
#else

Filter::Filter(ros::NodeHandle *nh) {
    pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("/points/filtered2", 1);
    tfBuffer_p = boost::make_shared<tf2_ros::Buffer>();
    tfListener_p = boost::make_shared<tf2_ros::TransformListener>(*tfBuffer_p);
    ros::Duration(2.0).sleep();

    cloud_lock_p = new boost::mutex();
    
    fl_received=new bool(false);
    fr_received=new bool(false);
    l_received=new bool(false);
    r_received=new bool(false);
    b_received=new bool(false);

    cloud_fl = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud_fr = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud_l = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud_r = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud_b = boost::make_shared<sensor_msgs::PointCloud2>();
    
    ros::Duration(0.5).sleep();

    sub1 = nh->subscribe<sensor_msgs::PointCloud2>("/points/frontleft", 1, boost::bind(&Filter::callback, this, _1, cloud_fl, fl_received, "frontleft"));
    sub2 = nh->subscribe<sensor_msgs::PointCloud2>("/points/frontright", 1, boost::bind(&Filter::callback, this, _1, cloud_fr, fr_received, "frontright"));
    sub3 = nh->subscribe<sensor_msgs::PointCloud2>("/points/left", 1, boost::bind(&Filter::callback, this, _1, cloud_l, l_received, "left"));
    sub4 = nh->subscribe<sensor_msgs::PointCloud2>("/points/right", 1, boost::bind(&Filter::callback, this, _1, cloud_r, r_received, "right"));
    sub5 = nh->subscribe<sensor_msgs::PointCloud2>("/points/back", 1, boost::bind(&Filter::callback, this, _1, cloud_b, b_received, "back"));

    ros::Duration(0.5).sleep();

    timer = nh->createTimer(ros::Duration(0.2), &Filter::callback_tim, this);
    ROS_INFO("Publishing the filtered point cloud on topic '/points/filtered2'");
}

void Filter::callback(const sensor_msgs::PointCloud2ConstPtr& msg, sensor_msgs::PointCloud2Ptr& cloud_pointer, bool*& status, const char* name) {
    if (!*status){
        ROS_INFO("Received data from %s camera", name);
    }

    //transform PointCloud2 to body frame
    sensor_msgs::PointCloud2 cloud_out;
    geometry_msgs::TransformStamped transform;
    try{
        transform = tfBuffer_p->lookupTransform("body", msg->header.frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast <float> ();
    boost::lock_guard<boost::mutex> lock(*cloud_lock_p);
    pcl_ros::transformPointCloud(mat, *msg, *cloud_pointer);
    cloud_pointer->header.frame_id = "body";
    *status = true;
}

void Filter::callback_tim(const ros::TimerEvent&) {
    boost::lock_guard<boost::mutex> lock(*cloud_lock_p);
    if ((!(*fl_received)) && (!(*fr_received)) && (!(*l_received)) && (!(*r_received)) && (!(*b_received))){
        ROS_WARN("No data");
        return;
    }

    // concatenate clouds
    pcl::PCLPointCloud2* cloud_cat = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtr(cloud_cat);
    if (*fl_received){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_fl, *cloud);
        *cloud_cat += *cloud;
    }
    if (*fr_received){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_fr, *cloud);
        *cloud_cat += *cloud;
    }
    if (*l_received){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_l, *cloud);
        *cloud_cat += *cloud;
    }
    if (*r_received){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_r, *cloud);
        *cloud_cat += *cloud;
    }
    if (*b_received){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_b, *cloud);
        *cloud_cat += *cloud;
    }
    
    // Perform the actual filtering
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud_filtered);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.05, 0.05, 0.02);
    sor.filter(*cloud_filtered);

    // Crop with bounding box
    geometry_msgs::TransformStamped transform;
    try{
        transform = tfBuffer_p->lookupTransform("body", "gpe", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    pcl::PCLPointCloud2 cloud_cropped;
    pcl::CropBox<pcl::PCLPointCloud2> bbox;
    double z_off = abs(transform.transform.translation.z);
    bbox.setMin(Eigen::Vector4f(-3.0, -3.0, -1.2+z_off, 1.0));
    bbox.setMax(Eigen::Vector4f(3.0, 3.0, 1.2-z_off, 1.0));
    bbox.setInputCloud(cloudPtr2);
    bbox.filter(cloud_cropped);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_cropped, output);

    // Transform cloud to vision frame
    geometry_msgs::TransformStamped transform2;
    sensor_msgs::PointCloud2 cloud_out;
    try{
        transform2 = tfBuffer_p->lookupTransform("vision", "body", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    Eigen::Matrix4f mat2 = tf2::transformToEigen(transform2).matrix().cast <float> ();
    pcl_ros::transformPointCloud(mat2, output, cloud_out);
    cloud_out.header.frame_id = "vision";

    // Publish the data
    pcl_pub.publish(cloud_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_filter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Filter pcl_filter = Filter(&nh);
    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();
}
#endif