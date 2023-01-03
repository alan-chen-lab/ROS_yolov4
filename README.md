# ROS_yolov4
### Env require:
> ubuntu 20.04 LTS   
> ROS Noetic  
> OpenCV-python 4.5.4  
### create the workspace
```
1. mkdir -p ~/your_ws_name
2. git clone git@github.com:alan-chen-lab/ROS_yolov4.git
3. cd ..   
4. catkin_make
```     
### Check your camera
```
ls /dev/video*
```
### Adjust yolov4.launch
> Path: cd src/robot_vision/launch/yolov4.launch
### Change default to your camera index
```
<arg name="camera_index" default="your camera index" />
```
