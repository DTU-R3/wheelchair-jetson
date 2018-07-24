# Propeller code for wheelchair
The package publishes padbot odometry based on encoders counts and integrates packages needed for padbot scenario into one launch file.

## Prerequisites
ROS Kinetic

## Installation
```
sudo apt install ros-kinetic-rosserial*
```

## Run the code

### Run propeller codes

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

### Run the whole scenario
roslaunch wheelchair_jetson wheelchair_jetson.launch 
