#include "traversability_builder.h"
using namespace std;

#if PCL_VERSION_COMPARE(<, 1, 10, 0)
#error "PCL of version at least 1.10 is required"
#else

Filter::Filter(ros::NodeHandle *nh) {
    pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("/traversability", 1);
    tfBuffer_p = boost::make_shared<tf2_ros::Buffer>();
    tfListener_p = boost::make_shared<tf2_ros::TransformListener>(*tfBuffer_p);
    ros::Duration(0.5).sleep();

    cloud_lock_p = new boost::mutex();
    grid_received=new bool(false);

    grid_received=new bool(false);
    trav = boost::make_shared<sensor_msgs::PointCloud2>();
    last_grid = boost::make_shared<sensor_msgs::PointCloud2>();
    
    ros::Duration(0.5).sleep();

    sub = nh->subscribe<sensor_msgs::PointCloud2>("/spot/no_step/raw", 1, &Filter::callback, this);
    ros::Duration(0.5).sleep();
    timer = nh->createTimer(ros::Duration(0.2), &Filter::callback_tim, this);
    ROS_INFO("Publishing the combined traversability map on topic '/traversability'");
}

void Filter::callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //transform PointCloud2 to vision frame
    sensor_msgs::PointCloud2 cloud_out;
    geometry_msgs::TransformStamped transform;
    try{
        transform = tfBuffer_p->lookupTransform("vision", msg->header.frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast <float> ();
    mat(2,3) = mat(2,3)+20.0;  // shift the point cloud up, such that positive and negative values fit into single voxel
    boost::lock_guard<boost::mutex> lock(*cloud_lock_p);
    pcl_ros::transformPointCloud(mat, *msg, *last_grid);
    last_grid->header.frame_id = "vision";
}

void Filter::callback_tim(const ros::TimerEvent&) {
    // concatenate clouds
    pcl::PCLPointCloud2* cloud_cat = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtr(cloud_cat);
    pcl::PCLPointCloud2* cloud_pcl = new pcl::PCLPointCloud2;
    if (*grid_received){
        pcl_conversions::toPCL(*trav, *cloud_pcl);
        *cloud_cat += *cloud_pcl;
    }
    pcl_conversions::toPCL(*last_grid, *cloud_pcl);
    *cloud_cat += *cloud_pcl;
    
    // Perform the actual filtering
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.05, 0.05, 1000.0);
    sor.filter(cloud_filtered);

    // Crop with bounding box
    pcl::PCLPointCloud2 cloud_cropped;
    pcl::CropBox<pcl::PCLPointCloud2> bbox;
    bbox.setMin(Eigen::Vector4f(-1000.,-1000.,10.,1.));
    bbox.setMax(Eigen::Vector4f(1000.,1000.,30.,1.));
    bbox.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(cloud_filtered));
    bbox.filter(cloud_cropped);

    // Convert to ROS data type and save for next iteration
    pcl_conversions::fromPCL(cloud_cropped, *trav);

    // Revert the shift applied in callback
    sensor_msgs::PointCloud2 cloud_out;
    Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();
    shift(2,3)=-20.;
    pcl_ros::transformPointCloud(shift, *trav, cloud_out);

    // Publish the data
    pcl_pub.publish(cloud_out);
    *grid_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traversability_builder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Filter pcl_filter = Filter(&nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}
#endif
