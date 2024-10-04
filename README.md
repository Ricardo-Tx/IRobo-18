# IRobo-18

## Guide

### What to do with `calculate_error.py` ?
Paste the improved version of the file in `<workspace>/src/turtlebot3_datasets/scripts/`.
Current improvements include:
- Publishing the error values to the **/error** topic
- Exiting in *SIGINT* a.k.a. exiting is faster


### What to do with `ekf_config.yaml` ?

The file should be copied to the directory `<workspace>/src/robot_localization/params/`. It is accessed by the `turtlebot3_playbag.launch` launch file dynamically at this location.

### What to do with `turtlebot3_playbag.launch` ?

The file should be copied to `<workspace>/src/turtlebot3_datasets/launch/` in order for the command 
```bash
roslaunch turtlebot3_datasets turtlebot3_playbag.launch
``` 
to work. It depends on `ekf_config.yaml` being in the right location and a RViz config file `rosbag.rviz` being available at the location `<workspace>/src/turtlebot3_datasets/data/`. Tweak the file locations in the launch file as necessary.

### What to do with the matrix transformation in the [turtlerobot3_datasets](https://github.com/irob-labs-ist/turtlebot3_datasets) package?

The transformation is strictly equal to that in `publish_initial_tf.sh`. The first 3 digits are the translation *(x, y, z)* and the last 4 are the rotation quaternion *(x, y, z, w)*.

### Running `source devel/setup.bash` constantly is boring
Copy `catkin_daemon.sh` to your home directory (adjacent to `.bashrc`).
Run
```bash
chmod +x catkin_daemon.sh
```
to make it executable. Then append the following line to `.bashrc`:
```bash
source catkin_daemon.sh
```
Now everytime you `cd` into a catkin workspace you will be prompted to automatically set it up if it's not already setup. It also warns about libraries without binaries in `devel/lib/` that need to be compiled with `catkin_make`.

### How to run `.launch` files?
Either `cd` into the location of the file and run
```bash
roslaunch my-launch.launch
``` 
or get the file into a `<pkg>/launch/` folder and run
```bash
roslaunch <pkg> my-launch.launch
```

### How to make gazebo not show GUI when running `roslaunch turtlebot3_gazebo turtlebot3_world.launch` ?
Run
```bash
cd `rospack find turtlebot3_gazebo`/launch
```
to move into the folder that contains all `.launch` files. Open the file `turtlebot3_world.launch` and edit the following line: 
```
...
<arg name="gui" value="true"/>
...
```