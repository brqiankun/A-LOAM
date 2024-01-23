# A-LOAM
## Advanced implementation of LOAM

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. It is a good learning material for SLAM beginners.

<img src=./picture/kitti.png width = 55% height = 55%/>

**Modifier:** [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```


## 4. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER and set the `dataset_folder` and `sequence_number` parameters in `kitti_helper.launch` file. Note you also convert KITTI dataset to bag file for easy use by setting proper parameters in `kitti_helper.launch`. 

```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
    roslaunch aloam_velodyne kitti_helper.launch
```
<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti_gif.gif" width = 720 height = 351 />

## 5. Docker Support
To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/A-LOAM/docker
make build
```
The build process may take a while depends on your machine. After that, run `./run.sh 16` or `./run.sh 64` to launch A-LOAM, then you should be able to see the result.


## 6.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).


里程计初步发布的为当前帧相对于里程计坐标系的位姿变换，因为里程计不知道世界坐标系和里程计坐标系的关系

Eigen::SelfAdjointEigenSolver 是什么

aloam中低频率的，精度较高的位姿变换也通过tf /aft_mapped发送出来了
-
直接使用第804到855行的发送逻辑，同一发送用于octomap建图

为何octomap建图建到1半卡住了？？？？


里程计初步发布的为当前帧相对于里程计坐标系的位姿变换，因为里程计不知道世界坐标系和里程计坐标系的关系

Eigen::SelfAdjointEigenSolver 是什么

aloam中低频率的，精度较高的位姿变换也通过tf /aft_mapped发送出来了
-
直接使用第804到855行的发送逻辑，同一发送用于octomap建图

为何octomap建图建到1半卡住了？？？？

patchworkpp 在重新播放rosbag时会自动停止卡死
la3dm也会发生同样的问题。
应该是ros的问题
将代码整合到一起来启动调试

### TODO
gpoctomap_server.cpp 在51行卡死
调低建图延迟时间
16线雷达建图有点太稀疏

为何分开启动aloam可以，而整合启动不可以

the ROS client libraries can listen to the /clock topic that is used to publish "simulation time".  
Time API for accessing time and sleeping instead of using the language-native routines. 
Normally, the ROS client libraries will use your computer's system clock as a time source, also known as the "wall-clock" or "wall-time" (like the clock on the wall of your lab).
When you are running a simulation or playing back logged datd a, it is often desirable to instead have the system use a simulated clock.

if the `/use_sim_time` parameter is set, the ROS Time API will return time=0, until it has received a value from the `/clock` topic. The time will only updated on receipt of a message from the `/clock` topic, and will stay constant between updates.
A Clock Server is any node that publishes to the /clock topic, and there should never be more than one running in a single ROS network. In most cases, the Clock Server is either a simulator or a log playback tool.
