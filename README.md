# Part of the code of orbbec final competition

The drone code of using Intel Realsense T265, Orbbec Gemini2 and PX4.

## Run

On Ubuntu 20.04 with ROS1 Noetic.

Make sure you have Docker, ROS and MAVROS installed.

### Compile

```shell
mkdir ~/ros_ws && cd ~/ros_ws
git clone --depth 1 https://github.com/EnderMandS/orbbec_2.git src
cd src
git clone --depth 1 -b frame_id_prefix https://github.com/EnderMandS/mavros.git
cd ~/ros_ws
source /opt/ros/noetic/setup.sh
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Run

Start code by running:
```shell
roslaunch basic_tf start.launch
```

### Docker images

```shell
docker pull endermands/ego-swarm:latest
docker pull endermands/orbslam3:latest
docker run --itd --name orbslam3 --net=host endermands/orbslam3:latest
docker run --itd --name ego_swarm --net=host endermands/swarm:latest
```

## Configuration

TODO.
