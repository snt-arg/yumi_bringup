# robotstudio_ros_bringup

## Requirements

```
sudo apt-get install python3-catkin-tools
sudo apt-get install python3-vcstool
```

## ROS Installation

The procedure is copied from [abb_robot_driver github](https://github.com/ros-industrial/abb_robot_driver)

```
# Change to the root of the Catkin workspace.

cd $HOME/catkin_ws
mkdir -p src

# Use vcstool to clone all required repositories into the 'src' space.

vcs import src --input https://github.com/ros-industrial/abb_robot_driver/raw/master/pkgs.repos

# Check build dependencies.

# First update the local rosdep database.

rosdep update

# And then ask rosdep to install any missing dependencies for us.

# NOTE: This may install additional packages, depending on the software installed

# on the machine. Be sure to check each command rosdep wants to run.rosdep install --from-paths src --ignore-src --rosdistro melodic

# Finally build the workspace (using catkin_tools).

catkin build

source $HOME/catkin_ws/devel/setup.bash
```

## RobotStudio Setup

Follow these [instructions](https://forums.robotstudio.com/discussion/12082/using-robotwebservices-to-access-a-remote-virtual-controller) to add your remote ROS environment IP to RobotStudio Virtual Controller's Whitelist

## Examples

Look [here](https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_robot_bringup_examples)

## Useful links

[A good github account for ABB Dev stuff](https://github.com/rparak)

