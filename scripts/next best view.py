#!/usr/bin/env python
# cd ~/workspaces/demo/src/spot_ros/spot_garbage_collector/scripts
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import numpy as np

class NextBestViewpoint:
    def __init__(self):
        rospy.init_node('next_best_viewpoint')
        self.grid_sub = rospy.Subscriber('map', OccupancyGrid, self.grid_callback)
        #self.grid_sub = rospy.Subscriber('/spot/occ_grid', OccupancyGrid, self.grid_callback)
        
        self.best_viewpoint_pub = rospy.Publisher('best_viewpoint', PointStamped, queue_size=1)
        self.best_viewpoint_pub_1 = rospy.Publisher('best_viewpoint_1', PointStamped, queue_size=1)

    def grid_callback(self, msg):
        global msg_g
        msg_g = msg
        grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # NVB 1        
        # Perform your algorithm to find the next best viewpoint here
        best_viewpoint_x, best_viewpoint_y = self.find_best_viewpoint(grid_data)

        # Create a PointStamped message for publishing the best viewpoint
        best_viewpoint_msg = PointStamped()
        best_viewpoint_msg.header = msg.header
        best_viewpoint_msg.point.x = best_viewpoint_x
        best_viewpoint_msg.point.y = best_viewpoint_y
        best_viewpoint_msg.point.z = 0.0

        # Publish the best viewpoint
        self.best_viewpoint_pub.publish(best_viewpoint_msg)

        # NVB 2        
        # Perform your algorithm to find the next best viewpoint here
        best_viewpoint_x_1, best_viewpoint_y_1 = self.find_best_viewpoint_1(grid_data)

        # Create a PointStamped message for publishing the best viewpoint
        best_viewpoint_msg_1 = PointStamped()
        best_viewpoint_msg_1.header = msg.header
        best_viewpoint_msg_1.point.x = best_viewpoint_x_1
        best_viewpoint_msg_1.point.y = best_viewpoint_y_1
        best_viewpoint_msg_1.point.z = 0.0

        # Publish the best viewpoint
        self.best_viewpoint_pub_1.publish(best_viewpoint_msg_1)

    def find_best_viewpoint(self, grid_data):
        # find the first unoccupied cell
        for i in range(grid_data.shape[0]):
            for j in range(grid_data.shape[1]):
                if grid_data[i, j] == 0:
                    return j * msg_g.info.resolution + msg_g.info.origin.position.x, i * msg_g.info.resolution + msg_g.info.origin.position.y
                
    def find_best_viewpoint_1(self, grid_data):
        # find the first unoccupied cell
        for i in range(grid_data.shape[0]):
            for j in range(grid_data.shape[1]):
                if grid_data[i, j] == 1:
                    return j * msg_g.info.resolution + msg_g.info.origin.position.x, i * msg_g.info.resolution + msg_g.info.origin.position.y

        # If no unoccupied cell found, return the center of the grid
        return msg_g.info.origin.position.x + (msg_g.info.width / 2) * msg_g.info.resolution, msg_g.info.origin.position.y + (msg_g.info.height / 2) * msg_g.info.resolution

if __name__ == '__main__':
    next_best_viewpoint = NextBestViewpoint()
    rospy.spin()