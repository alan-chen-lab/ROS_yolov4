# ROS_yolov4
## Env require:
> ubuntu 20.04 LTS   
> ROS Noetic  
> OpenCV-python 4.5.4  
## Step 1 : git clone this workspace
## Step 2 : catkin_make
> cd workspace & catkin_make
     
## Check your camera
> ls /dev/video*
## Setup camera to yolo
### Adjust yolov4.launch
> Path: cd src/robot_vision/launch/yolov4.launch
### Change default to your camera index
```
<arg name="camera_index" default="your camera index" />
```
