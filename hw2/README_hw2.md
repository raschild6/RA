# HW2

## Preparation
Substitute the file ar_ur5.srdf inside /ros_ws/src/internal/ar_no_gripper_moveit_config/config/ with the one provided

## Usage

To launch the first task open a shell and type

```bash
roslaunch ar_arena ar_bringup.launch simulation:=true gripper_enable:=false
```
to open the Gazebo simulator without the gripper.
Then on another shell type

```bash
roslaunch ar_arena apriltag.launch simulation:=true
```
to start the apriltag detection.
On another shell type
```bash
export ROS_NAMESPACE=/ur5
rosrun rviz rviz
```
to open Rviz and add the Planning Scene to visualize it. The export is required to set the prefix of the UR5 manipulator on topics.
#### Task 1
On a new shell type 
```bash
export ROS_NAMESPACE=/ur5
rosrun hw2 node_hw2_task_1 [0/1] frame_id_0 frame_id_1...
```
to run the pick and place of desired objects where the first argument is set to 0 to use the PCL detection or 1 to use the AprilTag detection.
