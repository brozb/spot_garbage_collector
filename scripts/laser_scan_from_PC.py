#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
import math
from tf import TransformBroadcaster
from rospy import Time 

def pointcloud_to_laserscan(pointcloud):
    angle_min = -math.pi # Minimum angle of the laser scan
    angle_max = math.pi   # Maximum angle of the laser scan
    angle_increment = math.pi / 270  # Angular resolution of the laser scan
    range_min = 0.0  # Minimum range value of the laser scan
    range_max = 100.0  # Maximum range value of the laser scan

    num_points = int((angle_max - angle_min) / angle_increment) + 1
    ranges = [range_max] * num_points

    for p in point_cloud2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
    
        if p[2] > 0.0:  # Only consider points with positive Z coordinate (above the ground)
            angle = math.atan2(p[1], p[0])
            index = int((angle - angle_min) / angle_increment)
            distance = math.sqrt(p[0] ** 2 + p[1] ** 2)
            #print("distance",distance)
            #print("ooo..",ranges[index])
            if distance < ranges[index]:
                ranges[index] = distance
     

    scan_msg = LaserScan()
    scan_msg.header = pointcloud.header
    scan_msg.angle_min = angle_min
    scan_msg.angle_max = angle_max
    scan_msg.angle_increment = angle_increment
    scan_msg.range_min = range_min
    scan_msg.range_max = range_max
    scan_msg.ranges = ranges

    return scan_msg

def pointcloud_callback(pointcloud):
    scan_msg = pointcloud_to_laserscan(pointcloud)
    #a = TransformBroadcaster()
    #a.sendTransform((0,0,0), (0,0,0,1), Time.now() - rospy.Duration(3) , 'base_link', 'map')
    scan_publisher.publish(scan_msg)


if __name__ == '__main__':
    rospy.init_node('scan')
    #pointcloud_topic = "/points"  # Replace with the actual point cloud topic
    pointcloud_topic = "/pcl_filtered"  # Replace with the actual point cloud topic
    scan_topic = "/scan"  # Replace with the desired laser scan topic

    pointcloud_subscriber = rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)
    scan_publisher = rospy.Publisher(scan_topic, LaserScan, queue_size=100)

    rospy.spin()
