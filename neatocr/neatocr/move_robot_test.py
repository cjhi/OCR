"""TESTING DOC"""

import easyocr
import PIL.Image
if not hasattr(PIL.Image, 'Resampling'):  # Pillow<9.0
    PIL.Image.Resampling = PIL.Image


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import time
from numpy import inf
import pandas as pd
import numpy as np
from std_msgs.msg import String


class move_robot(Node):
    def __init__(self):
        super().__init__("move_robot")
        print("I am Robert. Hello.")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        #self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)
        self.read_sign_sub = self.create_subscription(String, "sign_text", self.process_text, 10)


    #def process_scan(self, scan):

    def process_text(self, anything):
        print(anything.data)

    def choosePath(self):
        msg = Twist()
        z_value = 0.0
        x_value = 0.3

        msg.angular.z = z_value
        msg.linear.x = x_value
        self.vel_pub.publish(msg)
        #print("moving forward at 0.3m/s (hopefully)")
        time.sleep(1) 
    
    #function for turning until there is nothing in front of the robot (via lidar scan)
    # def turnUntilClear(self, straightAhead, generalLeftScan, generalRightScan):
    #     msg = Twist()
    #     msg.angular.z = 0.0
    #     msg.linear.x = -0.2
    #     self.vel_pub.publish(msg)
    #     print("moving backward")
    #     time.sleep(1.5)
        
    #     if min(generalLeftScan) < min(generalRightScan):
    #         print("turning right until clear")
    #         msg.angular.z = -0.3
    #     else:
    #         print("turning left until clear")
    #         msg.angular.z = 0.3
    #     msg.linear.x = 0.0
    #     self.vel_pub.publish(msg)
    #     time.sleep(3) #consider making this pause very short/ so continuously updating
    
    def run_loop(self):
        self.choosePath()

def main(args=None):
    #print("main reached")
    rclpy.init(args=args)
    robert = move_robot()
    # robert.choosePath()
    rclpy.spin(robert)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
