#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from spot_msgs.srv import SetVelocityRequest
from spot_msgs.srv import SetVelocity

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

def velocity_limit_client():
    print("Sevice.")
    rospy.wait_for_service('/spot/velocity_limit')

    try:
        # Create a service proxy to call the service
        service_proxy = rospy.ServiceProxy('/spot/velocity_limit', SetVelocity)
        
        # Create a request object
        request = SetVelocityRequest()
        request.velocity_limit.linear.x = 0.2
        request.velocity_limit.linear.y = 0.2
        request.velocity_limit.angular.z = 0.2
        

        # Call the service and store the response
        response = service_proxy(request)
        
        # Process the response
        if response.success: 
            print("success")
        else:
            print("not")

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", str(e))

def callback(data):
    global publisher
    # Process the clicked point
    point = data.point
    rospy.loginfo("Received clicked point: x = %f, y = %f, z = %f", point.x, point.y, point.z)
    # Create a new PoseStamped message
    pose = PoseStamped()

    # Fill in the pose information
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'vision'
    pose.pose.position.x = point.x
    pose.pose.position.y = point.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    # Publish the message
    publisher.publish(pose)


def clicked_point_subscriber():
    global publisher
    rospy.init_node('clicked_point_subscriber', anonymous=True)

    # Define the message type and callback function
    topic = '/clicked_point'
    rospy.Subscriber(topic, PointStamped, callback)

    topic2 = '/spot/go_to_pose'
    publisher = rospy.Publisher(topic2, PoseStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':

    velocity_limit_client()

    #Set_spot_vel = rospy.ServiceProxy("/spot/velocity_limit", SetVelocity)
    #Set_spot_vel(0.2, 0.2, 0 , 0, 0 , 0.2)
    
    clicked_point_subscriber()


