# Drone code

Part of the code of ORBBEC final competition.

The drone code of using Intel RealSense T265, ORBBEC Gemini2 and PX4.

It connects orbslam3, ego-swarm and PX4.

# Start

On Ubuntu 20.04 with ROS1 Noetic.

## Compile

```shell
mkdir ~/ros_ws && cd ~/ros_ws
git clone --depth 1 --recursive https://github.com/EnderMandS/orbbec_2.git src
cd src
git clone --depth 1 --recursive -b master https://github.com/mavlink/mavros.git
cd ~/ros_ws
source /opt/ros/noetic/setup.sh
catkin_make -DCMAKE_BUILD_TYPE=Release
```

For MAVROS compile, you may need to install some other packages by `apt install`.

## Run

Before running the code, make sure you have ego-swarm and orbslam3 docker container, [see below](# Docker). Starting the code by running:
```shell
source ~/ros_ws/devel/setup.zsh
roslaunch basic_tf start.launch
```

## Docker

### Pull images

```shell
docker pull endermands/ego-swarm:latest
docker pull endermands/orbslam3:latest
```

For mainland China:

```shell
docker pull ccr.ccs.tencentyun.com/endermands/ego-swarm:latest
docker pull ccr.ccs.tencentyun.com/endermands/orbslam3:latest
```

### Run

```shell
docker run --itd --name orbslam3 --net=host endermands/orbslam3:latest
docker run --itd --name ego_swarm --net=host endermands/swarm:latest
```

You can now attach shell to the container and run `roslaunch`. For more detail, please refer to [ego-swarm](https://github.com/EnderMandS/ego-swarm) and [orb-slam3-ros](https://github.com/EnderMandS/orb_slam3_ros).

Attach shell to the `orbslam3` container and run:

```shell
source ~/ros_ws/devel/setup.zsh
roslaunch orb_slam3_ros orbbec_rgbd.launch
```

Attach shell to the `ego_swarm` container and run:

```shell
source ~/code/ros_ws/devel/setup.zsh
roslaunch ego_planner run.launch
```

# Configuration

For multi drone system, modify the `drone_id` of the `start.launch` file in the `basic_tf` package and also in docker container's launch file.

Also modify `px4_config.yaml` odometry plugin parameter: 

```yaml
# odom example
odometry:
  fcu:  # change frame_id to drone_0/...
    map_id_des: "drone_0/map"    # desired parent frame rotation of the FCU's odometry
    odom_child_id_des: "drone_0/base_link"    # desired child frame rotation of the FCU's odometry
    odom_parent_id_des: "drone_0/odom"
```

## Camera calibration

Please refer to [Kalibr](https://github.com/ethz-asl/kalibr).

# Note

- As it has some ROS message timestamp bug when IMU rate >100Hz. DO NOT set ORBBEC gemini2 camera IMU rate greater than 100Hz.
- DO NOT enable both T265 fisheyes ROS topics and ORBBEC color and depth ROS topics. Choose one or the other, otherwise the USB bus will be congested. This happens on both the Intel NUC and the NVIDIA Jetson Orin nano.
