# Part of the code of orbbec final competition

This is part of 4th 3DV final competition code. For other parts of code, please refer to the modified [ego planner](https://github.com/EnderMandS/ego-planner) and [ORB-SLAM3](https://github.com/EnderMandS/orb_slam3_ros).

## Run

On Ubuntu 20.04 with ROS1 Noetic.

Make sure you have Docker, ROS and MAVROS installed.

#### Compile

```shell
mkdir ~/ros_ws && cd ~/ros_ws
git clone --depth 1 https://github.com/EnderMandS/orbbec_2.git src
source /opt/ros/noetic/setup.sh
catkin_make
```

#### Get docker images and run

```shell
docker pull endermands/ego_planner:latest
docker pull endermands/orbslam3:latest
docker run --itd --name orbslam3 --net=host endermands/orbslam3:latest
docker run --itd --name ego_planner --net=host endermands/ego_planner:latest
```

