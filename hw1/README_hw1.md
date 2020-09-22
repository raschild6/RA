# HW1

## Usage

To launch the arena in the Gazebo simulator open a shell and type

```bash
roslaunch ar_arena ar_bringup.launch simulation:=true
```
Then on another shell type

```bash
roslaunch ar_arena apriltag.launch simulation:=true
```
to start the apriltag detection.

#### Task 1
On a new shell type 
```bash
rosrun hw1 node_hw1_task_1 frame_id_0 frame_id_1 ...
```
to run the AprilTag detection and stamp the pose of desired object

#### Task 2
On a new shell type
```bash
rosrun hw1 node_hw1_task_2 [0/1] frame_id_0 frame_id_1 ...
```
to run the PCL detection of object passed as arguments, where the first argument is a 1 if the visualization of clouds is activated and 0 otherwise. If omitted, first argument is 0.
