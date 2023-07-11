#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from spot_msgs.srv import SetVelocityRequest
from spot_msgs.srv import SetVelocity
from std_srvs.srv import Trigger
import tf2_ros
import numpy as np

""" SET LIMIT SPEED OF THE SPOT FIRST
rosservice call /spot/velocity_limit "velocity_limit:
  linear:
    x: 0.2
    y: 0.2
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2" 
success: True
message: "Success"
"""


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    return float(np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb)))


class PoseMove:
    def __init__(self):
        rospy.init_node('clicked_point_subscriber', anonymous=True)
        rospy.loginfo("Waiting for '/spot/velocity_limit' service")
        rospy.wait_for_service('/spot/velocity_limit')

        rospy.loginfo("Waiting for '/spot/stop' service")
        rospy.wait_for_service('/spot/stop')

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(0.5)

        try:
            # Create a service proxy to call the service
            self.service_proxy = rospy.ServiceProxy('/spot/velocity_limit', SetVelocity)

            # Create a request object
            request = SetVelocityRequest()
            request.velocity_limit.linear.x = 0.4
            request.velocity_limit.linear.y = 0.4
            request.velocity_limit.angular.z = 1.0

            # Call the service and store the response
            response = self.service_proxy(request)

            # Process the response
            if response.success:
                rospy.loginfo("Velocity limits set")
            else:
                rospy.logerr("Setting velocity limits failed")
                rospy.signal_shutdown("Unable to set velocity limits")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            rospy.signal_shutdown("Unable to set velocity limits")

        try:
            # Create a service proxy to call the service
            self.stop_proxy = rospy.ServiceProxy('/spot/stop', Trigger)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            rospy.signal_shutdown("Stop service not available")

        topic2 = '/spot/go_to_pose'
        self.publisher = rospy.Publisher(topic2, PoseStamped, queue_size=10)

        topic = '/clicked_point'
        self.subs = rospy.Subscriber(topic, PointStamped, self.callback)

        rospy.loginfo("Set the goal in rviz, timeout for the move is 5 seconds")
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        # stop robot on shutdown
        self.subs.unregister()
        rospy.sleep(0.2)
        rospy.loginfo("Stopping robot")
        self.stop_proxy()
        rospy.sleep(0.5)

    def callback(self, data):
        # Process the clicked point
        point = data.point
        rospy.loginfo("Received clicked point: x = %f, y = %f, z = %f", point.x, point.y, point.z)
        # Create a new PoseStamped message
        pose = PoseStamped()

        # Find current robot position
        try:
            trans = self.tfBuffer.lookup_transform(data.header.frame_id, 'body', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            return

        # Calculate vector between current pose and the goal, get yaw in the goal frame
        p = np.array([[trans.transform.translation.x], [trans.transform.translation.x], [0]])
        g = np.array([[point.x], [point.y], [0]])
        direction = g-p
        x_axis = np.array([[1], [0], [0]])
        z_axis = np.array([[0], [0], [1]])
        yaw = get_angle(x_axis, direction, z_axis)
        print(yaw)
        qz = np.sin(yaw/2.)
        qw = np.cos(yaw/2.)

        # Fill in the pose information
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = data.header.frame_id
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # Publish the message
        self.publisher.publish(pose)


if __name__ == '__main__':
    pose_move = PoseMove()
    rospy.spin()
    