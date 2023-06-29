#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_py as tf2
import numpy as np
import ros_numpy
import genpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


class Cloud2Grid:
    def __init__(self):
        rospy.init_node('cloud2grid', anonymous=True)
        self.odom = 'vision'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.2)

        self.cloud_subs = rospy.Subscriber('/traversability', PointCloud2, self.cloud_callback, queue_size=1)

        self.grid_pub = rospy.Publisher('/occ_grid', OccupancyGrid, queue_size=1)

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

    def cloud_callback(self, msg):
        points = np.transpose(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg))

        origin = np.min(points, axis=1, keepdims=True)
        origin[2, 0] = 0.  # origin just in xy plane
        resolution = 0.1

        xy = points[:2, :]-origin[:2, :]
        xy = ((xy + resolution / 2) // resolution).astype(int)
        height = int(np.max(xy[1, :])+1)
        width = int(np.max(xy[0, :])+1)

        unknown = np.ones((width,height))
        unknown[xy[0,:],xy[1,:]]=0
        value_grid = np.ones((width, height))
        value_grid[xy[0, :], xy[1, :]] = points[2, :]

        grid = np.zeros((width,height))
        grid[value_grid>=0] = 0
        grid[value_grid<0] = 99
        grid[unknown==1] = -1
        grid_p = np.pad(grid,((1,1),(1,1)),'constant',constant_values=((-1,-1),(-1,-1)))
        width_p = width+2
        height_p = height+2

        grid_l = grid_p.T.reshape((width_p*height_p)).astype(int).tolist()
        o = OccupancyGrid()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.odom
        o.info.map_load_time = msg.header.stamp
        o.info.resolution = resolution
        o.info.width = width_p
        o.info.height = height_p
        o.info.origin.position.x = origin[0, 0]-resolution
        o.info.origin.position.y = origin[1, 0]-resolution
        o.info.origin.orientation.w = 1.
        o.data = grid_l
        self.grid_pub.publish(o)


if __name__ == '__main__':
    p = Cloud2Grid()
    rospy.spin()
