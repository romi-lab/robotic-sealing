# User manual for this computer

```
Written by Maggie Xu 22 Jan 2020
```
This docuamnetation records all the relatred programs on this computer,
their fucntionality, command , explaination, etc. Related settings such as calibration and visualization are also included.

## Table of content

- [1. Overview](#1-overview)
- [2. Setup](#2-setup)
  * [2.1 Setup UR robot](#21-setup-ur-robot)
  * [2.2 Calibration](#22-calibration)
    + [2.2.1 Camera Calibration](#221-camera-calibration)
      - [1.Realsense RGB-D](#1realsense-rgb-d)
      - [2.Web cam (logi C525)](#2web-cam--logi-c525-)
    + [2.2.2 Eye-hand calibration](#222-eye-hand-calibration)
    + [2.2.3 Tool calibration](#223-tool-calibration)
  * [2.3 Construct model for visualization](#23-construct-model-for-visualization)
    + [2.3.1 Desing of mounting (Solidworks to stl)](#231-desing-of-mounting--solidworks-to-stl-)
    + [2.3.2 Constrcut model in rviz (stl to urdf)](#232-constrcut-model-in-rviz--stl-to-urdf-)
  * [2.4 Setup sealing system](#24-setup-sealing-system)
- [3.Robotic sealing](#3robotic-sealing)
    + [3.1 Explaination of important parameters in the code](#31-explaination-of-important-parameters-in-the-code)
      - [1.seam_detection_line.py](#1seam-detection-linepy)
      - [2.measure_error.py](#2measure-errorpy)
      - [3.seam_tracking.py](#3seam-trackingpy)
    + [3.2 Revert to robotic welding](#32-revert-to-robotic-welding)
- [4. Other functions](#4-other-functions)
    + [1. Record rgb & depth image and point cloud](#1-record-rgb---depth-image-and-point-cloud)
    + [2. Open Kinect camera in ROS](#2-open-kinect-camera-in-ros)
    + [3. Record current pose of UR](#3-record-current-pose-of-ur)
    + [Other tools](#other-tools)
- [5.Documents in this project](#5documents-in-this-project)
  * [1.CNERC](#1cnerc)
  * [2.Project Docs](#2project-docs)
- [6.Note](#6note)
  * [Summary of some command](#summary-of-some-command)


# 1. Overview
This computer is used to develop system for robotics welding and sealing.
The system is mainly developed using ROS melodic with Ubuntu 18.04.
Visualization is achieved by Rviz.
Motion control is achieved using URx, it can also be controlled by Moveit. This computer used dual system: Ubuntu18.04 and Windows.
```
Password for ubuntu(default): space(keyboard)
Password for windows: 1314
```


Most of the program can be found under folder ```/home/maggie/ros```

A rough description of useful packages is listed here:

Folder | package | Description
------------ | ------------- | -------------
**robotic_sealing** | seam_tracking | The main program, includes seam detection and visual feedback based seam tracking. Refer to **3. Robotic sealing** for detail
&nbsp;| sealing_robot_description | The model for robotic sealing (used in rviz)
&nbsp;| robotic-welding-hri | HRI control of UR using Leap Motion
**ur_driver** | Universal_Robots_ROS_Driver | driver for UR
&nbsp; |fmauch_universal_robot | provide model of UR and moveit control of UR
rviz_config | | store configuration files for visualization in rviz
tool | StructureSDK | SDK for Structure core with ROS driver
&nbsp; | iai_kinect | ROS driver for kinect v2    
&nbsp; |ar_track_alvar | detect AR marker (normally used for calibration)
&nbsp; | average_tf | average poses of four AR marker to get a more accurate pose (written by Muddassir)
&nbsp; | easy_handeye | Hand-eye calibration
&nbsp; |ros_record_rgbd_images| record rgbd image and point cloud of camera
&nbsp; |markers| markers for calibration (AR and chess board)
testing | ur_tests | test UR
&nbsp;| moveit_tutorials | test & learn Moveit
&nbsp;| panda_moveit_config | test & learn Moveit
&nbsp;| edge_detection_packages | some trail packages to test edge detection function
Kalman and Bayesian Filters in Python |  | learn Kalman filter, cloned from [here](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

# 2. Setup
Setup of the system mainly includes:

Part | Description
------------ | -------------
2.1 setup UR robot | connect UR with teach pendant
2.2 calibration | camera, tool, eye-hand calibration
2.3 construct model for visualization | integrate the model and visualise it in rviz
2.4 setup sealing system | mounting design and pneumatic control

## 2.1 Setup UR robot
In this part, we mainly need to connec the driver with UR, so it can controlled and visualised on a real-time basis in ROS. Without the need of visualization, UR can be controlled indepenedly with URx python.  

The driver is installed at ```/home/maggie/ros/ur_driver```   
The full description of this package can be found at [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)  
The process of setting up UR in this computer is documented at ```/home/maggie/notes/1.2 Setup UR with Moveit.md```

Important information for setup:
```
UR Network(Static address):
IP: 192.168.0.2
Mask: 255.255.255.0
Gate: 192.168.0.10

Host(PC's IP for external control):
IP:192.168.0.3
```

On the teach pendant, you may need to set payload (normally it's 2kg)
After installation, UR robot can be connect in ROS by running
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
```

## 2.2 Calibration
### 2.2.1 Camera Calibration
#### 1.Realsense RGB-D
Use realsense SDK to calibrate it.  
SR305 depth_to_color_extrinsics:
```
frame_id: "depth_to_color_extrinsics"
rotation: [0.9999980330467224, -0.0012284178519621491, 0.0015572066185995936, 0.001234484720043838, 0.9999916553497314, -0.0039010262116789818, -0.001552401459775865, 0.0039029407780617476, 0.9999911785125732]
translation: [0.025699999183416367, 0.00042214873246848583, 0.003923953045159578]
```

To open camera in ROS, run
```
roslaunch seam_tracking rs_camera.launch
```

#### 2.Web cam (logi C525)
Cheese board can be found at ```/home/maggie/ros/tool/src/markers/Cheeseboard.pdf```.
To open it in ROS, run
```
roslaunch seam_tracking logi_camera.launch
```
To open both camera, run
```
roslaunch seam_tracking camera.launch
```

### 2.2.2 Eye-hand calibration
The code and explanation can be found at
[Hand-eye calibration](https://github.com/IFL-CAMP/easy_handeye)
In our case, we use AR marker which can be found at ```/home/maggie/ros/tool/src/markers/Markers_0_to_8-converted.pdf``` You may need to measure the lenght of marker and specify it in the launch file.

```
# Open realsesne camera
roslaunch seam_tracking rs_camera.launch

# Start AR marker detection, modify this file to chaneg the length of marker
roslaunch ar_track_alvar pr2_indiv_no_kinect.launch

# Average four markers detected, you can specify the marker number in this file
roslaunch average_tf average_tf.launch

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml

roslaunch ur5_moveit_config execution_real.launch

roslaunch easy_handeye eye-in-hand.launch

# subscribe to the rgb image to check if the marker of interest is inside the view
rqt_image_view
```

The process should be similar to [here](https://youtu.be/cXCfv0dmoAA) Theorically the more poses, the more accurate it gets. 12 or more poses are recommanded.  The final result is the transfromation from camera to end-effector. It can be used directly in seam detection, under the function ```def transform_cam_wrt_base(pcd, T_end_effector_wrt_base)```

### 2.2.3 Tool calibration
Follow the tutorial video [here](https://www.youtube.com/watch?v=7m6dif8LDhY)

## 2.3 Construct model for visualization
The program of this part is stored in ```/home/maggie/ros/robotic_sealing/src/sealing_robot_description```

Part | Description
------------ | -------------
kinect2_description | model of kinect
realsense2_description | model of realsense
ur_description | model of UR
robot_scene_description | integration of all model and add mounting stl

### 2.3.1 Desing of mounting (Solidworks to stl)
The  mounting is drawn in Solidworks 2016 in Window and export as stl
The drawing file can be found [here](https://grabcad.com/library/robotic-sealing-mounting-config-1)

### 2.3.2 Constrcut model in rviz (stl to urdf)
After getting the stl, put it under folder ```robot_scene_description/meshes/collision```  
The descirption of the scene is under folder ```robot_scene_description/urdf```:
```
sealing_robot.urdf.xacro: sealing scene description (default)
weldingrobot.urdf.xacro: welding scene description
```
To run the visualization:
```
# connect to UR before visualization
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml

# visualize it
roslaunch robot_scene_description robot_scene_description.launch
```
By default, it opens the sealing_robot.urdf.xacro.
If you need to change it to welding, change ```robot_scene_description/launch/planning_context.launch```
```
<!-- Load universal robot description format (URDF), for sealing-->
<param if="$(arg load_robot_description)" name="$(arg robot_description)" command="xacro --inorder  '$(find robot_scene_description)/urdf/weldingrobot.urdf.xacro'"/>
```

## 2.4 Setup sealing system
The system used a pneumatic gun powered by a pump with max P = 6 Mpa. It is connected to a solenoid which can then be controlled by URX
```
robot = urx.Robot("192.168.0.2")
# Open
robot.set_digital_out(0,True)
# Close
robot.set_digital_out(0,False)
```
The mounting stl file is placed at ```/home/maggie/ros/robotic_sealing/src/sealing_robot_description/robot_scene_description/meshes/collision/sealing_mounting.STL``` (refer to 2.3 Construct model for visualization for detail)

After installation the mounting is fixed and two important parameters are listed below:
```
1.hand-eye transfromation:
T_cam_wrt_end_effector = np.array([[-0.02160632 ,-0.97207334 ,-0.23368052,  0.122350972619],
                                  [ 0.98357004,  0.02123446, -0.17927374, -0.08164344],
                                  [ 0.17922931 ,-0.2337146   ,0.95564342 ,  0.1156235],
                                  [ 0.         , 0.      ,    0.         ,1.        ]])

2.tcp:
tcp_sealing_gun = [-0.0002, -0.09216, 0.32202, 0, 0, 0]
```

# 3.Robotic sealing
Several functions are developed here. The code can be found at ```/home/maggie/ros/robotic_sealing/src/``` and explainations on [here](https://github.com/romi-lab/robotic-sealing)

Package | Description
------------ | -------------
seam_tracking | main program
sealing_robot_description | refer to **2.3 Construct model for visualization**
robotic-welding-hri |  HRI control of UR using Leap Motion (developed by Ali) check [Demo](https://youtu.be/xw_W_4LYhmI)

Under ```seam_tracking```, the programs are organised as so:

Folder | Program | Description
------------ | ------------- | -------------
seam_detection | seam_detection_line.py | Seam detection, path planning and execution for line shape groove (Plane, horizonal, vertical, beam, cube) ([Demo](https://youtu.be/HzMZ8QXGl44))
&nbsp;|seam_detection_others.py | Seam detection, path planning and execution for non-line type of groove (tube)
&nbsp;|seam_detection_rest.py | Seam detection, path planning and execution for non-line type of groove (tube & cube intersection)
feedback_control | find_seam.py | seam_detection and path planning, the code highly resembles ```seam_detection/seam_detection_line.py```, **the different is that this file doesn't execute the path, it passed the path to ```feedback_control/seam_tracking.py``` for feedback motion control**
&nbsp;| measure_error.py | the new way to measured the error, check result [here](https://youtu.be/ZCArchHNFno)
&nbsp;| seam_tracking.py | feedback motion control based on result from ```feedback_control/measure_error.py``` and ```feedback_control/find_seam.py```
image_processing | logi_img_processing.py | the old way of image processing using logi camera, the result can be found [here](https://youtu.be/ECuXRa2A77M) (upper left image)
&nbsp;| rs_rgb_processing.py| the old way of image processing using realsense rgb camera, the result can be found [here](https://youtu.be/ECuXRa2A77M) (lower right image)
old_feedback_control | demo_detection.py | seam detection and path planning, this file doesn't execute the path either (similar to find_seam.py), it passed the path to ```feedback_control/new_rgb_cam_control.py``` for feedback motion control (check demo [here](https://youtu.be/LGgVzwvURLI))
&nbsp;| demo_motion_control.py | feedback control based on the old way of image processing using both cameras ( ```image_processing/rs_rgb_processing.py``` and ```image_processing/logi_img_processing.py```) (check demo [here](https://youtu.be/LGgVzwvURLI))
others | &nbsp; | for store & match template used in ```seam_detection_rest.py```

To run the codes:

1.Detection ([demo](https://youtu.be/HzMZ8QXGl44)):
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
roslaunch robot_scene_description robot_scene_description.launch
roslaunch seam_tracking rs_camera.launch

cd /home/maggie/ros/robotic_sealing/src/seam_tracking/script/seam_detection
python seam_detection_line.py
# or python seam_detection_others.py depends on the workpiece
```
2.Detection with feedback control ([demo](https://youtu.be/ZCArchHNFno))
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
roslaunch robot_scene_description robot_scene_description.launch
roslaunch seam_tracking camera.launch

# in a new terminal
cd ~/ros/robotic_sealing/src/seam_tracking/script/feedback_control
python measure_error.py
python seam_tracking.py
python find_seam.py
```
3.Detection with the old way of feedback control using two camera ([demo](https://youtu.be/LGgVzwvURLI))
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
roslaunch robot_scene_description robot_scene_description.launch
roslaunch seam_tracking camera.launch
roslaunch seam_tracking demo.launch

python /home/maggie/ros/robotic_sealing/src/seam_tracking/script/old_feedback_control/demo_detection.py
python /home/maggie/ros/robotic_sealing/src/seam_tracking/script/old_feedback_control/demo_motion_control.py
```

### 3.1 Explaination of important parameters in the code
#### 1.seam_detection_line.py
Check [here](https://github.com/romi-lab/robotic-welding-demo) for expalnation.

Parameter | Location | Description
------------ | ------------- | -------------
startj   | line 944 | the start pose of ur for capturing point cloud
execution | line 945 | execution = True means the program may execute the planned path, set it to false to skip execution for debugging
max_dis | line 946 | point cloud exceeds max distance will be disregarded
mutilayer_exe | line 947 | indicate if mutilayer operation is needed
sealing | line 948 | when sealing == True, it rotates the planned path by 30 degree along y axis for non-blockage injection of plasters
voxel_size | line 902 | downsample voxel size for processing point cloud
neighbor | line 905 | parameter for finding clusters using dbscan method (line 467)
delete_percentage  | line 906 | the line with feature value below this percentage will be delete, the left will be used for clustering and path planning_context
thin_line | line 62  | parameter in path planning, control the radius in which points are selected to perform principle component analysis. Visit [here](https://github.com/aliadnani/3D-Point-Cloud-Curve-Extraction) for detail
sorted_point_distance | line 109 | parameter in path planning, control the radius adjacent points are selected for ordering the thinned curve. Visit [here](https://github.com/aliadnani/3D-Point-Cloud-Curve-Extraction) for detail
offset_z/y/x | line 728 | slighly adjust the pose for execution, xyz with respect to the tool (poses') orientation

#### 2.measure_error.py
Measure the error using rgb web camera mainly based on Canny edge detection and Hough transfromation.

Parameter | Location | Description
------------ | ------------- | -------------
tip_width, tip_height | line 105, 106 | The location of tip in the image
left_rectangle_center | line 164 | the position of ROI for detecting the edge_detection_packages
left_offset | line 239 | this distance between the seam to the detected edge
edges | line 323 | Canny dege detection threshold
lines | line 324 | threshold indicates minimum length to form a line (in pixel)
n | line 514 | number of poses for averaging in low-pass filter
alpha | line 516 | how much the original data need to be fixed
new_a1-a1 | line 525 | the threshold for difference between current pose and the filtered one

Since the line detected are mainly vertical here, I filtered the line based on two points at edges of ROI instead of k,b of line function.

#### 3.seam_tracking.py
Feedback control of robot. Received first pose from find_seam.py and feedback from measure_error.py
The control is linear based on velocity in tool coordinate. Two parameters are used: error between target and current tip pose for centering, angle of line for aligning camera with it.

Parameter | Location | Description
------------ | ------------- | -------------
vel_scale | line 80 | velocity of the overall system to move along the seam, also used for feedback control
residual | line 182 | the threshold for residul indicates when the robot stop working and when it approaches to the end

### 3.2 Revert to robotic welding
In case you need to revert the system for robotic welding. You need to:
```
1.chaneg the model for visualization:
Refer to 2.3.2 Constrcut model in rviz (stl to urdf)

2.change hand-eye transfromation:
T_cam_wrt_end_effector = np.array([[            1,   2.7798e-18,           0,  -0.01032],
                                   [ -5.57321e-19,            1,   0.0197993,  -0.12237],
                                   [   1.0842e-19,            0,           1,   0.07477],
                                   [           0,             0,           0,         1] ])

3.change tcp:
tcp_welding_torch = [-0.00072, 0.05553, 0.2312, -0.8504775921315857, -0.02340453557068149, -0.015929517346989313]
```

The previous demo for welding can be found at ```/home/maggie/Documents/Robotic welding & sealing/Video/robotic_welding.mp4```

# 4. Other functions
I record other fucntions that may be useful during the development of system
### 1. Record rgb & depth image and point cloud
Code under folder ```/home/maggie/ros/tool/src/ros_record_rgbd_images```
```
# to record point cloud
python /home/maggie/ros/tool/src/ros_record_rgbd_images/cloud_conversion_and_save.py

# to record rgb and depth images
# press 'a' at opencv window to capture
python /home/maggie/ros/tool/src/ros_record_rgbd_images/record_color_depth_images_to_disk.py -c /camera/color/image_raw -d /camera/aligned_depth_to_color/image_raw

# to record point cloud and rgb and depth images
/home/maggie/ros/tool/src/ros_record_rgbd_images/point_cloud_and_rgbd.py -c /camera/color/image_raw -d /camera/aligned_depth_to_color/image_raw
```

### 2. Open Kinect camera in ROS
```
roslaunch seam_tracking kinect.launch
# use rviz or rqt_image_view to visualise it if you want
```

### 3. Record current pose of UR
```
# to get the correct pose, set the tcp correctly
python /home/maggie/ros/testing/src/ur_tests/record_ur_pose.py
```

### 3. View transformation relation
```
rosrun tf view_frames

# rosrun tf tf_echo [reference_frame] [target_frame]
rosrun tf tf_echo camera_color_optical_frame camera_depth_optical_frame
```

### 4. Change transformation
```
python /home/maggie/ros/tool/src/easy_handeye/easy_handeye/scripts/calculate_tansformation.py
```

### Other tools

Other tools can be found under ```/home/maggie/tools```

Folder | Description
------------ | -------------
libfreenect2 | kinect sdk
librealsense | realsense camera sdk
StructureSDK-CrossPlatform-0.8.1 | Structure core sdk
rtl88x2BU_WiFi_linux_v5.3.1_27678.20180430_COEX20180427-5959-master | driver for wifi adaptor, detail check ```/home/maggie/notes/wifi.md```

Some useful softare

Name | Description
------------ | -------------
Kazam | Screen record
Openshot | video editor
cheese | open camera and capture
Solidworks 2016 (in Windows) | CAD drawing

# 5.Documents in this project
## 1.CNERC
Porgress report and annula report can be found under folder ```/home/maggie/Documents/CNERC```
## 2.Project Docs
Under folder ```/home/maggie/Documents/Robotic welding & sealing```  
More project videos for sealing can also be found on [youtube](https://youtube.com/playlist?list=PL5FErTrn43DDHdJKLiQ1JPjlA0XReVGos)

# 6.Note

## Summary of some command
**Open cameras**
```
# realsesne
roslaunch seam_tracking rs_camera.launch
or
roslaunch realsense2_camera rs_camera.launch
# web cam
roslaunch seam_tracking logi_camera.launch
or
roslaunch usb_cam usb_cam-test.launch
# kinect
roslaunch seam_tracking kinect.launch
```
**Visualization**
```
# connect to UR before visualization
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
# visualize it
roslaunch robot_scene_description robot_scene_description.launch
```
