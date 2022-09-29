# ROS_yolov4
## Step 1 : git clone this workspace
## Step 2 : catkin_make
> cd workspace & catkin_make
     
## Check your camera
> ls /dev/video*
## Setup camera to yolo
> cd src/robot_vision/launch/yolov4.launch
```
<arg name="camera_index" default="your video index" />
```
