# OCR
Robot direction-taking with OCR for ROS2

## Features
### Optical Character Recognition
Our package establishes a live video feed to a capable machine and converts any words in the image to text.
![A video of a sign that reads "Turn right" held in front of a live video feed. A terminal window shows a stream of output: "Reading text from image! Turn"](webgraphics/scansign.gif)
### Sign Following
Our package enables our robot to follow instructions written on signs around a space, enabling it to travel to different locations and complete missions!
```python
#Alan Please Add Details
```
### Additional Features
Our package has additional features coming soon!
```python
#Vis a vis
```

## Install Instructions
### Install EasyOCR
```bash
pip install easyocr
```
If troubleshooting you may want to use these [Install Instructions](https://github.com/JaidedAI/EasyOCR/tree/master#installation). 

### Install OpenCV
If you are confident you have one, stable release of OpenCV 4.x installed on your machine you can skip these steps.
```bash
pip uninstall easyocr
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
# Create build directory
mkdir -p build && cd build
# Configure
cmake  ../opencv-4.x
# Build
cmake --build .
```
Which come from [OpenCV 4.x](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)'s install instructions.

### Install prerequesites
```bash
sudo apt-get update && sudo apt-get install -y ros-humble-gazebo-ros-pkgs \
	ros-humble-nav2-bringup \
	ros-humble-navigation2 \
	ros-humble-camera-info-manager \
	ros-humble-cartographer-ros \
	ros-humble-cartographer \
	ros-humble-gscam \
	git \
	python3-colcon-common-extensions \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-ugly \
	gstreamer1.0-libav gstreamer1.0-tools \
	gstreamer1.0-x \
	gstreamer1.0-alsa \
	gstreamer1.0-gl \
	gstreamer1.0-gtk3 \
	gstreamer1.0-qt5 \
	gstreamer1.0-pulseaudio \
	python3-pip \
	hping3
```
Create a ROS2 workspace and download our dependencies.
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/comprobo23/neato_packages
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Clone our package
```bash
cd ~/ros2_ws/src
git clone git@github.com:cjhi/OCR.git
```
