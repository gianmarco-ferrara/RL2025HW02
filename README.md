# RL2025HW02 :package:
Control your robot

## Getting Started :hammer:

```shell
cd ~/ros2_ws
git clone https://github.com/gianmarco-ferrara/RL2025HW02.git
colcon build 
source install/setup.bash
```
## Usage :white_check_mark:

 ## **1. Kinematic control** :checkered_flag:
 ### 1.1 Control with `ros2_kdl_node`:
Launch the `iiwa.launch.py` from the `iiwa_bringup` package with the desired command interface and controller:
```shell
ros2 launch iiwa_bringup iiwa.launch.py command_interface:=<cmd_interface> robot_controller:=<controller>
```
where `<cmd_interface>` is `velocity` or `position` and 
`<controller>` is `velocity_controller` or `iiwa_arm_controller`.
<br>  
In another terminal launch the `iiwa.control.launch.py` from the `ros2_kdl_package` package with the desired command interface and control mode:
```shell
ros2 launch ros2_kdl_package iiwa.control.launch.py cmd_interface:=<cmd_interface> ctrl:=<ctrl>
```
where 
`<cmd_interface>` is `velocity` or `position` and 
`<ctrl>` is `velocity_ctrl` or `velocity_ctrl_null`.

 ### 1.2 Control with action server
Launch the `iiwa.launch.py` from the `iiwa_bringup` package with the desired command interface and controller:
```shell
ros2 launch iiwa_bringup iiwa.launch.py command_interface:=<cmd_interface> robot_controller:=<controller>
```
where `<cmd_interface>` is `velocity` or `position` and 
`<controller>` is `velocity_controller` or `iiwa_arm_controller`.
<br>  
Run the server with the right command interface in a second terminal:
```shell
ros2 run ros2_kdl_package kdl_action_server --ros-args -p cmd_interface:=<cmd_interface>
```
where 
`<cmd_interface>` is `velocity` or `position`.

<br>

Run the client (in a third terminal) that will send requests to the server passing the parameters from the `parameters.yaml` file with the desired command interface and control mode:
```shell
ros2 launch ros2_kdl_package iiwa.client.launch.py cmd_interface:=<cmd_interface> ctrl:=<ctrl>
```
where 
`<cmd_interface>` is `velocity` or `position ` and 
`<ctrl>` is `velocity_ctrl` or `velocity_ctrl_null`.

 ## **2. Vision-based control** :camera:
Launch the `iiwa_world_vision.launch.py` from the `ros2_kdl_package` package with the desired command interface and controller:
```shell
ros2 launch ros2_kdl_package iiwa_world_vision.launch.py
```

This command will load the `rqt_image_view` node and the `single.launch.py` from the `aruco_ros` package.

<br>

In another terminal run the `ros2_kdl_node_vision` from the `ros2_kdl_package` in order to start the vision control:
```shell
ros2 run ros2_kdl_package ros2_kdl_node_vision
```

<br>

It is possible to move the `arucotag` object by calling the corresponding remapped service from gazebo:
```shell
ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose "{
  entity: {
    name: 'arucotag',  
    type: 1           
  },
  pose: {
    position: {x: 1.0, y: 1.5, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```
### Note:
In the plot folder codes to convert from `.db3` files to `.csv` and plot them are available :coffee:
