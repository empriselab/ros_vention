# Setting up Vention Navigation Stack

# Navigation Dependencies
Create a ROS workspace in your home directory:
```
mkdir vention_dependencies_ws
```
We use Cartographer for multi-lidar SLAM. Follow their instructions at https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html.
Be sure to set this up in vention_dependencies_ws.

Source vention_dependencies_ws before continuing.

# ros_vention

Download the Vention ROS package, and put it into catkin_ws.

Download the URDF at
```
https://drive.google.com/file/d/1OZAdcuAua0Nr7p6ITxTQDwUMFzZjeR8F/view?usp=sharing
```
Put the URDF into ros_vention/urdf/meshes.

build the workspace with

```
catkin build
```


# Teleoperation with Xbox
```
ros_vention/src/controllers/vention_joystick_control_v2.py
```

# Navigation 

## Load Vention RobotModel
```
roslaunch ros_vention vention_description.launch
```

## Start Lidar
You may need to change the usb id/path of lidars in the launch file.
```
roslaunch ros_vention vention_rplidar_a1.launch
```

## Start Realsense
We are using the Realsense built-in IMU. If we have a true IMU, we don't need this.
We use the IMU for odom -> vention_base_link.
```
roslaunch ros_vention vention_odm_d435.launch
```

## Start Cartographer For SLAM

We use cartographer for map -> odom
```
roslaunch ros_vention vention_cartographer_lidar.launch
```

## Start move_base For Navigation
You may need to change the Arduino usb id cmd_vel_bridge_basicmicro.py.
```
roslaunch ros_vention vention_navigation.launch
```

## Rviz

Open RViz and use the config in rviz/vention.rviz

You can move the base by giving it a 2D nav goal in RViz.
