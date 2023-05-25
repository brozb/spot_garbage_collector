#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_py as tf2
import numpy as np
import ros_numpy
import genpy
from sensor_msgs.msg import PointCloud2

class Processor:
    def __init__(self):
        rospy.init_node('scan_processor', anonymous=True)
        self.odom = 'vision'
        self.body = 'body'
        self.clouds = {}
        self.stamps = {}
        self.freq = rospy.get_param('/cloud_freq')
        self.cloud_timeout = 3.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.2)

        self.fl_subs = rospy.Subscriber('/points/frontleft', PointCloud2, self.cloud_callback, 'frontleft',queue_size=1)
        self.fr_subs = rospy.Subscriber('/points/frontright', PointCloud2,self.cloud_callback, 'frontright',queue_size=1)
        self.l_subs = rospy.Subscriber('/points/left', PointCloud2,self.cloud_callback, 'left',queue_size=1)
        self.r_subs = rospy.Subscriber('/points/right', PointCloud2,self.cloud_callback, 'right',queue_size=1)
        self.b_subs = rospy.Subscriber('/points/back', PointCloud2,self.cloud_callback, 'back',queue_size=1)

        self.cloud_pub = rospy.Publisher('/points', PointCloud2,queue_size=1)
        self.clean_pub = rospy.Publisher('/points/clean', PointCloud2,queue_size=1)

        self.tim = rospy.Timer(rospy.Duration(1/self.freq), self.cat_clouds)

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        """invoked on rospy shutdown signal, shutdown all timers and stop the robot

        :return: None
        """
        self.tim.shutdown()
        rospy.sleep(0.1)

    def get_transform(self, tf_from, tf_to, out="matrix", time=None, dur=0.1):
        """returns the latest transformation between the given frames
        the result of multiplying point in frame tf_to by the output matrix is in the frame tf_from

        :param tf_from: find transform from this frame
        :param tf_to: find transform to this frame
        :param out: the return type
                    - 'matrix' - returns numpy array with the tf matrix
                    - 'tf' - returns TransformStamped
        :param time: the desired timestamp of the transform (ROS Time)
        :param dur: the timeout of the lookup (float)
        :return: as selected by out parameter or None in case of tf2 exception
                    - only ConnectivityException is logged
        """
        if time is None:
            tf_time = rospy.Time(0)
        else:
            if not isinstance(time, rospy.Time) and not isinstance(time, genpy.Time):
                raise TypeError("parameter time has to be ROS Time")
            tf_time = time

        try:
            t = self.tf_buffer.lookup_transform(tf_from, tf_to, tf_time, rospy.Duration(dur))
        except (tf2.LookupException, tf2.ExtrapolationException):
                return None
        except tf2.ConnectivityException as ex:
            rospy.logerr(ex)
            return None
        
        # return the selected type
        if out == "matrix":
            return ros_numpy.numpify(t.transform)
        elif out == "tf":
            return t
        else:
            raise ValueError("argument out should be 'matrix' or 'tf'")

    def cloud_callback(self, msg, position):
        """convert the point cloud to numpy array, transform it to odom frame and save to the dictionary"""
        t1 = self.get_transform(self.body, msg.header.frame_id, time=msg.header.stamp)
        t2 = self.get_transform(self.odom, self.body, time=msg.header.stamp)
        t3 = self.get_transform(self.body, 'gpe', time=msg.header.stamp)
        z_offset = np.abs(t3[2,3])

        points = np.transpose(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg))
        points = np.concatenate((points, np.ones((1, np.shape(points)[1]))), axis=0)
        points_tf = np.matmul(t1, points)
        
        #apply bounding box
        a = np.abs(points_tf[0,:])<=3
        b = np.abs(points_tf[1,:])<=3
        c = points_tf[2,:]<=1.2-z_offset
        points_tf = points_tf[:, np.where(np.logical_and(a, np.logical_and(b, c)))[0]]

        #bring it to static frame
        points_tf = np.matmul(t2, points_tf)[0:3,:]

        self.clouds[position] = points_tf[0:3,:]
        self.stamps[position] = msg.header.stamp

    def cat_clouds(self, _):
        """concatenate the clouds from the individual depth cameras into one point cloud in odom frame
        takes the last available output for each camera"""
        if len(self.clouds) != 5:
            rospy.logwarn('Not all depth camera point clouds are available')
        for i in self.clouds:
            if rospy.Time.now().to_sec() - self.stamps[i].to_sec() >= self.cloud_timeout:
                rospy.logerr('No data from depth camera %s received for at least %f seconds' % (i, self.cloud_timeout))
        if len(self.clouds) == 0:
            rospy.logwarn('No data from depth cameras received yet')
            return

        points = np.array([[],[],[]])
        for i in self.clouds:
            points = np.concatenate((points, self.clouds[i]),axis=1)

        points_t = np.transpose(points)
        pc_array = np.zeros(len(points_t), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
        ])
        pc_array['x'] = points_t[:, 0]
        pc_array['y'] = points_t[:, 1]
        pc_array['z'] = points_t[:, 2]

        pc_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=rospy.Time.now(), frame_id=self.odom)

        self.cloud_pub.publish(pc_msg)
        

if __name__ == '__main__':
    p = Processor()
    rospy.spin()
