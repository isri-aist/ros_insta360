# Insta360 ONE X2 ROS PACKAGE

- cd ~/catkin_ws/src
- git clone https://github.com/NathanCrombez/insta360.git
- Edit the CMakeLists.txt (INSTA_SDK path)
- (optional) Edit the bringup.launch file with your camera(s) intrinsic paramters
- cd ~/catkin_ws
- catkin_make
- source ~/catkin_ws/devel/setup.bash
- roslaunch insta360 bringup.launch


- rqt (/insta360/right/image_raw & /insta360/left/image_raw):
![image](https://github.com/NathanCrombez/insta360/assets/25529025/a29441e9-a188-47bd-b7bd-a9b7be9a39b9)


- rostopic echo /insta360/right/camera_info:
```
header: 
  seq: 263
  stamp: 
    secs: 1699234777
    nsecs: 564966558
  frame_id: "/camera_right_optical_frame"
height: 560
width: 1152
distortion_model: "r1r2p1p2xi"
D: [0.0, 0.0, 0.0, 0.0, 1.63172]
K: [922.516889, 0.0, 574.28678, 0.0, 924.37605, 577.50644, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

- rostopic echo /insta360/left/camera_info:
```
header: 
  seq: 6174
  stamp: 
    secs: 1699236754
    nsecs: 868220593
  frame_id: "/camera_left_optical_frame"
height: 560
width: 1152
distortion_model: "r1r2p1p2xi"
D: [0.0, 0.0, 0.0, 0.0, 1.6023]
K: [905.63493, 0.0, 577.15881, 0.0, 907.6885, 579.63735, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

