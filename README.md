# Control your robot

## Available Packages in this Repository ##
- `ros2_iiwa` 
- `ros2_kdl_package`
- `aruco_ros`

## Getting Started
```shell
git clone https://github.com/danieffe/RoboticsLab-HM2.git
colcon build 
source install/setup.bash

```

# **Usage**

 ## **1. Launch the Manipulator in Rviz**
To visualize the manipulator and verify the URDF setup:

```shell
ros2 launch iiwa_description aruco_gazebo.launch.py robot_controller:=velocity_controller command_interface:=velocity
```
rviz will be started.


 ## **2. Launch the Manipulator in Gazebo**
To start the simulation including the ArUco marker environment::

```shell

ros2 launch iiwa_description aruco_gazebo.launch.py start_rviz:=false robot_controller:=velocity_controller command_interface:=velocity use_sim:=true
```
This will open Gazebo with the IIWA robot and the ArUco tag placed in front of the camera.


## **3. Start ArUco Detection**
Once Gazebo is running, open another terminal and launch the detection node:
```shell
ros2 launch aruco_ros single.launch.py
```
This node detects the marker in real time and publishes its pose on `/aruco_single/pose`.


## **4. Run the KDL Control Node**
The main control node can be launched with different control modes using parameters defined in config/kdl_params.yaml
```shell
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=<controller_type>
```

where `controller_type` can be:<br>
`velocity_ctrl` — Standard velocity control<br>
`velocity_ctrl_null` — Null-space control with joint limit avoidance<br>
`vision_ctrl` — Vision-based control using ArUco detection

Alternatively, run from launch file with custom parameters :

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=<controller_type>
```


## **5. Action Server and test client**
A trajectory can be executed asynchronously via the integrated Action Server:
```shell
ros2 action send_goal /execute_trajectory ros2_kdl_package/action/ExecuteTrajectory \
"{traj_duration: 25.0, acc_duration: 10.0, total_time: 25.0, kp: 1.0, end_position_x: 0.7, end_position_y: 0.0, end_position_z: 1.0}"
```

To test the server, the correspective test client can be executed:
```shell
ros2 run ros2_kdl_package trajectory_client
```

## **6. Adjust ArUco Marker Pose in Gazebo**
To reposition the ArUco marker during simulation (e.g., move it slightly left and tilt 60° toward the camera):
```shell
ros2 service call /world/aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'arucotag'},
  pose: {
    position: {x: -0.2, y: -0.63, z: 0.48},
    orientation: {x: 0.0, y: 0.5, z: 0.0, w: 0.866}
  }}"
```
