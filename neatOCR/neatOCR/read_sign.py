import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class read_sign(Node):
    def __init__(self, image_topic):
        super().__init__('read_sign')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.create_subscription(Image, image_topic, self.process_image, 10)

    #Taken from Neato Soccer Example code
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        

def main():
    print('Hi from neatOCR.')

if __name__ == '__main__':
    main()
