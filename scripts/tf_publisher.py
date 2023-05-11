#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time 

def main():
    rospy.init_node('my_broadcaster')
    
    b = TransformBroadcaster()
    c = TransformBroadcaster()
    
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    rate = rospy.Rate(100)  # 5hz
    
    x, y = 0.0, 0.0
    
    while not rospy.is_shutdown():
        if x >= 2:
            x, y = 0.0, 0.0 
        
        x += 0.01
        y += 0.01
        
        translation = (x, y, 0.0)
        translation1 = (x-1, y, 0.0)
        
        
        b.sendTransform(translation, rotation, Time.now(), 'ignite_robot', '/world')
        c.sendTransform(translation1, rotation, Time.now(), 'ignite_robot2', '/world')
        rate.sleep()
    


if __name__ == '__main__':
    main()