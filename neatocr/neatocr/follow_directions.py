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


class follow_directions(Node):
    def __init__(self):
        super().__init__("follow_directions")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        #self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)
        self.read_sign_sub = self.create_subscription(String, "sign_text", self.process_text, 10)
        
        #for storing current directions
        self.current_instructions = ""
        
        #for storing current Neato linear and angular velocities
        self.current_x = 0.0
        self.current_z = 0.0


    #def process_scan(self, scan):
        #to fill out if we end up adding LiDAR stuff

    def process_text(self, anything):
        print(anything.data)
        #will need to add something for processing multiple words
        instructions = anything.data
        self.current_instructions = instructions

    def choosePath(self):
        sleep_time = 0.1
        turn_time = 3 #amount of time it takes to turn 90 degrees? need to set/calibrate this
        print("true or false?")
        print (self.current_instructions != "")

        # Choose motor speeds based on instructions
        if (self.current_instructions != ""):
            print("passed first if statement")
            if self.current_instructions == "Go":
                print("instructions are Go")
                z_value = 0.0
                x_value = 0.3
                sleep_time = 0.1
            elif self.current_instructions == "Turn Right":
                z_value = -0.2
                x_value = 0.0
                sleep_time = turn_time
                print("instructions are Turn Right")
            elif self.current_instructions =="Turn Left":
                z_value = 0.2
                x_value = 0.0
                sleep_time = turn_time
                print("Instructions are Turn Left")
            elif self.current_instructions == "Stop":
                z_value = 0.0
                x_value = 0.0
                sleep_time = 0.1
                print("Instructions are Stop")
            else:
                print("no instructions found")
                pass
        else:
            z_value = self.current_z
            x_value = self.current_x
            sleep_time = 0.1

        self.current_z = z_value
        self.current_x = x_value

        # #for testing:
        # z_value = 0.0
        # x_value = 0.0
        
        #publish motor speeds and commands to robot
        print("publishing motor speeds to robot")
        msg = Twist()
        msg.angular.z = z_value
        msg.linear.x = x_value
        self.vel_pub.publish(msg)
        time.sleep(sleep_time) 
    
    def run_loop(self):
        #self.process_text(self)
        self.choosePath()
        pass

def main(args=None):
    print("main reached")
    rclpy.init(args=args)
    node = follow_directions()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
