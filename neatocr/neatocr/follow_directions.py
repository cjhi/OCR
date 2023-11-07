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
        #for storing instructions/ states
        self.current_instructions = ""
        self.last_instructions = ""
        self.boredom = 0
        self.next_sleep = 0.0

        #for storing current Neato linear and angular velocities
        self.current_x = 0.0
        self.current_z = 0.0

    #def process_scan(self, scan):
        #to fill out if we end up adding LiDAR stuff

    def process_text(self, anything):
        print(f"Latest text: {anything.data}")
        instructions = anything.data
        self.current_instructions = instructions

    def choosePath(self):
        #After sleeping in the loop, just publish a stop command.
        print(f"current: {self.current_instructions} last: {self.last_instructions}")
        self.drive(0.0,0.0)
        turn_time = 1.6 # number of seconds it takes to turn 90 degrees?
        if self.current_instructions == self.last_instructions: # Prevent a command from mattering forever
            self.current_x = 0.0
            self.current_z = 0.0
            self.next_sleep = 0.5
            self.boredom += 1
            if self.boredom >= 8: #Allow the command to work again after a few seconds, if it's still there.
                self.last_instructions = "null"
        else:         # Choose motor speeds based on instructions
            match self.current_instructions:
                case "go":
                    self.current_x = 0.2
                    self.current_z = 0.0
                    self.next_sleep = 3.0
                    print("instructions are to Go Forward")
                    self.last_instructions = "go"
                case "back up":
                    self.current_x = -0.12
                    self.current_z = 0.0
                    self.next_sleep = 2.2 
                    print("instructions are to Back Up")
                    self.last_instructions = "back_up"
                case "turn right":
                    print("not passing")
                    self.current_x = 0.0
                    self.current_z = -1.0
                    self.next_sleep = turn_time
                    print("instructions are to Turn Right")
                    self.last_instructions = "turn right"
                case "turn left":
                    self.current_x = 0.0
                    self.current_z = 1.0
                    self.next_sleep = turn_time
                    print("Instructions are to Turn Left")
                    self.last_instructions = "turn left"
                case "stop":
                    self.current_x = 0.0
                    self.current_z = 0.0
                    self.next_sleep = 0.5
                    print("Instructions are to Stop")
                    self.last_instructions = "stop"
                case _:
                    print("Clear skies, no rules found.")
                    self.current_instructions = "null"
        
        #publish motor speeds and commands to robot for the time allotted
        self.drive(x=self.current_x, z=self.current_z)
        print(f"linear {self.current_x}, angular {self.current_z}")
    
    def drive(self,x,z):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.vel_pub.publish(msg)
    
    def run_loop(self):
        time.sleep(self.next_sleep)
        self.choosePath() # Loop frequency determined with sleepTime in ChoosePath

def main(args=None):
    rclpy.init(args=args)
    node = follow_directions()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
