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
python src/ros_vention/src/controllers/basicmicro_arduino/vention_controller.py
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

## Start ZED Camera
We are using the ZED built-in VIO.
We use the IMU for odom -> vention_base_link.
```
roslaunch ros_vention vention_zed_pose.launch
```

## Start Cartographer For SLAM

We use cartographer for map -> odom

For building map:
```
roslaunch ros_vention vention_cartographer_lidar.launch
```
Save map using 
```
python src/ros_vention/scripts/build_map_interactive.py --pbstream-file /home/isacc/deployment_ws/src/ros_vention/maps/emprise_572_map.pbstream
```

For using against existing map:
```
python src/ros_vention/scripts/build_map_interactive.py --pbstream-file /home/isacc/deployment_ws/src/ros_vention/maps/emprise_572_map.pbstream
```

## Start move_base For Navigation

First make sure we are publishing the odom link required:

```
python src/ros_vention/scripts/zed_pose_to_odom_feedback.py
```

You may need to change the Arduino usb id cmd_vel_bridge_basicmicro.py.
```
roslaunch ros_vention vention_navigation.launch
```

## Rviz

Open RViz and use the config in rviz/vention.rviz

You can move the base by giving it a 2D nav goal in RViz.

## Capture Named Locations

```
python src/ros_vention/scripts/capture_named_locations.py --locations microwave --locations-file /home/isacc/deployment_ws/src/feeding-deployment/config/nav_named_locations.yaml
```

## [Feeding-Deployment] Test Navigation

Check that you are in feed conda env
```
python /home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/test_navigate_action.py
```


