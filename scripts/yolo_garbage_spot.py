#!/usr/bin/env python3

# Import libraries
# https://docs.ultralytics.com/tasks/
# python3 pub_camera.py
from ultralytics import YOLO

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from tf import TransformBroadcaster
from rospy import Time
import rospkg 

bridge = CvBridge()
#model = YOLO('yolov8n-seg.yaml')
# YOLO Parameters
# model = YOLO('yolov8n-seg.pt')
# model = YOLO('yolov8n.pt')
rospack = rospkg.RosPack()
path = rospack.get_path('spot_garbage_collector')+'/scripts/'
model = YOLO(path+'best.pt') 
current_frame = []

# TF Parameters
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0, 1.0)

def callback(data):
  global current_frame, pub, TF_a

  br = CvBridge()
  current_frame = br.imgmsg_to_cv2(data)
  current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

  results = model(current_frame)
  annotated_frame = results[0].plot()

  # We have to veriry if there are objects detected in the image, if not, we can't procced
  if not results[0]:
   
    print("No Detections....")
  
  else:

    #print("Num Clases Det",len(results[0].boxes))

    # Metadats extraction
    clase = int(results[0].boxes[0].cls[0])
    conf = float(results[0].boxes[0].conf[0])
    coord_x = results[0].boxes[0].xywh[0][0]
    coord_y = results[0].boxes[0].xywh[0][1]
    coord_w = results[0].boxes[0].xywh[0][2]
    coord_h = results[0].boxes[0].xywh[0][3]

    #translation = (coord_x/100, coord_y/100, -0.5)
    dist_prof = 3
    dist_ancho = 2
    pos_y = dist_ancho - (1/160)*(coord_x)
    pos_x = dist_prof - coord_w/160
    translation = (pos_x, pos_y, -0.45)
    TF_a.sendTransform(translation, rotation, Time.now(), 'detected_obj', '/body')

    for i in range(len((results[0].boxes))):

      clase = int(results[0].boxes[i].cls[0])
      conf = float(results[0].boxes[i].conf[0])
      coord_x = results[0].boxes[i].xywh[0][0]
      coord_y = results[0].boxes[i].xywh[0][1]

    
  # Publish the final image result
  pub.publish(br.cv2_to_imgmsg(annotated_frame))

def publish_message():
  global current_frame, pub, TF_a

  pub = rospy.Publisher('spot_image_CNN', Image, queue_size=10)
  rospy.init_node('video_pub_py', anonymous=True)

  #rospy.Subscriber('spot_image', Image, callback)
  rospy.Subscriber('/spot/camera/hand_color/image', Image, callback)

  rate = rospy.Rate(1000) # 10hz
  TF_a = TransformBroadcaster()
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():

      rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
