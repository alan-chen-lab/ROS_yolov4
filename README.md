# ROS_yolov4
## Env require:
> ubuntu 20.04 LTS   
> ROS Noetic  
> OpenCV-python 4.5.4  
## Step 1 : git clone and create the workspace
```
mkdir -p ~/your_ws_name/src
cd your_ws_name/src/    /   catkin_init_workspace
cd ..     /     catkin_make
```
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
