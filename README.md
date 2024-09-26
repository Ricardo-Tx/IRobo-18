# IRobo-18

## Guide

### How to run `.launch` files?
Either `cd` into the location of the file and run
```
roslaunch my-launch.launch
``` 
or get the file into a `<pkg>/launch/` folder and run
```
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