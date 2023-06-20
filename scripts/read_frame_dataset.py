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
import time, os


bridge = CvBridge()
#model = YOLO('yolov8n-seg.yaml')
# YOLO Parameters
# model = YOLO('yolov8n-seg.pt')

current_frame = []

# TF Parameters
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0, 1.0)

cont = 0
def callback(data):
  try:
    global current_frame, pub, TF_a, cont

    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    timestamp = int(time.time())
    cont += 1
    filename = f"dataset_spot_{cont}.jpg"

    # Directorio de destino
    dest_dir = "garbage dataset 2"
    # Comprobar si el directorio de destino existe, si no, crearlo
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
    
    dest_path = os.path.join(dest_dir, filename)

    cv2.imwrite(dest_path, current_frame)
    
    #cv2.imshow("image", current_frame)
    time.sleep(1.0) 
  except Exception as e:
    print(e)




def main():
  global current_frame, pub, TF_a

  pub = rospy.Publisher('spot_image_CNN', Image, queue_size=10)
  rospy.init_node('video_pub_py', anonymous=True)

  #rospy.Subscriber('spot_image', Image, callback)
  rospy.Subscriber('/spot/camera/hand_color/image', Image, callback)

  TF_a = TransformBroadcaster()
  br = CvBridge()

  rate = rospy.Rate(0.5)  # Set the desired rate to 0.5 Hz (2 seconds)

  while not rospy.is_shutdown():
      rate.sleep()

  rospy.spin()
  #    rate.sleep()

if __name__ == '__main__':
  main()