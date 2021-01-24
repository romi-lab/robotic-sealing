# Robotic sealing
This program is developed for visual based feedback control on robotic sealing. It partially adopted our previous work on robotic welding and explainations can be found [here](https://github.com/romi-lab/robotic-sealing)

## result
### Robotic Sealing

### Feedback control


More related project videos can be found on [youtube](https://youtube.com/playlist?list=PL5FErTrn43DDHdJKLiQ1JPjlA0XReVGos)

## overview

Package | Description
------------ | -------------
seam_tracking | main program
sealing_robot_description | Construct model for visualization in ROS
robotic-welding-hri |  HRI control of UR using Leap Motion 

## 1.robotic-welding-hri
Gestured based interaction for guiding. For detail , please go to [here](https://github.com/aliadnani/robotic-welding-hri)

## 2.sealing_robot_description
The mounting in CAD (Solidworks) avaliable at [GrabCAD](https://grabcad.com/library/robotic-sealing-mounting-config-1)

## 3.seam_tracking

The programs are organised as so:

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

## Note
Detail on the development of this program, please refer to the [manual](https://github.com/romi-lab/robotic-sealing/blob/main/manual/User%20Manual.md).
