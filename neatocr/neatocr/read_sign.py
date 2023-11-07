import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #Converts ros node images to openCV image objects for all other purposes
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

import easyocr
import PIL.Image
if not hasattr(PIL.Image, 'Resampling'):  # Pillow<9.0
    PIL.Image.Resampling = PIL.Image

reader = easyocr.Reader(['en']) # this needs to run only once to load the model into memory

class read_sign(Node):
    """ The OCR Camera is a Python object that encompasses a ROS node 
        that can process images from the camera and run EasyOCR on the image.
        The node will issue motor commands to move forward if and only if the
        # word "go" is detected. """

    def __init__(self, image_topic):
        """ Initialize the Camera node with OCR builtin """
        super().__init__('read_sign')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.command_pub = self.create_publisher(String, 'sign_text', 10)

        # Toggles the display of the camera feed with output on top of it
        thread = Thread(target=self.loop_wrapper)
        thread.start()

        # ocrthread = Thread(target=self.process_text)
        # ocrthread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_text(self):
        if not self.cv_image is None:
            result = reader.readtext(self.cv_image) #actual camera output
        # result = [["Stop","Go"],[1,2,3],[4,5,6]] # This is for test
            if (result != []):
                # msg = String(msg) # When testing just go [0][1] for go and [0][0] for stop
                # msg = String("Hello World!")
                result = self.extract_directive(result)
                # print(result) #debug print what the OCR is reading/detecting
                msg = String(data=result) # The piece d'resistance
                self.command_pub.publish(msg)

    def extract_directive(self, easyocr_output):
        cmd_options = np.array(['stop','go','back up','turn left','turn right'])
        words = np.array(['null']) #Initialize a array in Numpy so you can search in it
        for entry in easyocr_output:
            words = np.append(words, entry[1].lower()) #add each word from the image
        print(words) #gives the clean output of every word cluster detected
        indices = np.in1d(words, cmd_options)
        directive = words[np.argmax(indices)]
        print(directive)
        return directive
    
    def run_loop(self):
        # TODO: move process_text() to a separate thread somehow and make it asynchronous
        if not self.cv_image is None:
            self.process_text()
            # # self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            # #print(self.cv_image.shape)
            # cv2.imshow('video_window', self.cv_image)
            # # cv2.imshow('binary_window', self.binary_image)
            # if hasattr(self, 'image_info_window'):
            #     cv2.imshow('image_info', self.image_info_window)
            # cv2.waitKey(5) #Prints if a key is pressed at 200hz (5ms sleep)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2, aka the openCV window."""
        cv2.namedWindow('video_window')
        # cv2.namedWindow('binary_window') #Creates the B&W window 
        # cv2.namedWindow('image_info')
        # self.red_lower_bound = 0
        # cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_bound, 255, self.set_red_lower_bound) # Adds trackbar to the B&W window
        cv2.setMouseCallback('video_window', self.process_mouse_event) #Allows you to click the image and see a pixel BGR value
        while True:
            self.run_loop()
            # time.sleep(0.05) #The openCV windows will update at most 20 hz

    # def set_red_lower_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the red lower bound with the slider"""
    #     self.red_lower_bound = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

def main(args=None):
    print("GO!")
    # result = reader.readtext('src/OCR/examples/english.png') #Tests OCR pkg
    # print(result)
    # print(type(result))
    # if (result != None):
    #     print(result[0][1])
    rclpy.init()
    node = read_sign("/camera/image_raw")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()