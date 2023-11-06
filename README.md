# Insta360 ONE X2 ROS PACKAGE

- cd ~/catkin_ws/src
- git clone https://github.com/NathanCrombez/insta360.git
- Edit the CMakeLists.txt (INSTA_SDK path)
- Edit the bringup.launch File with your camera(s) intrinsic paramters
- cd ~/catkin_ws
- catkin_make
- source ~/catkin_ws/devel/setup.bash
- roslaunch insta360 bringup.launch

