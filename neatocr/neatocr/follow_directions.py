#Brooke and Swasti's person follower node
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
import string
from std_msgs.msg import String


class follow_directions(Node):
    def __init__(self):
        super().__init__("follow_directions")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)
        #to do: fix this subscription
        self.read_sign_sub = self.create_subscription(String, "text", self.process_text, 10)

    def process_scan(self, scan):
        #To do: implement
        pass

    def process_text(self, text):
        print(text)


    def choosePath(self):
        #To do: implement
        msg = Twist()

        z_value = 0
        x_value = 0
        msg.angular.z = z_value
        msg.linear.x = x_value
        self.vel_pub.publish(msg)
        time.sleep(0.1) 
    
    def run_loop(self):
        self.process_text(self)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = follow_directions()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
