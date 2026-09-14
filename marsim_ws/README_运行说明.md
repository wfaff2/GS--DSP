# MARSIM 三维激光雷达无人机示例

当前工作区使用 MARSIM 官方 `ubuntu20` 分支，适配本机的 Ubuntu 20.04 和
ROS Noetic。默认运行 CPU 版 MID-360 三维激光雷达仿真，不需要 Gazebo。

## 启动单无人机 MID-360 示例

```bash
cd "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch test_interface single_drone_mid360.launch
```

RViz 打开后，使用工具栏中的 `3D Goal`，先单击目标位置，再拖动鼠标指定方向，
无人机会飞向三维目标并持续生成局部雷达点云。

## 常用话题

```bash
rostopic list
rostopic hz /quad0_pcl_render_node/cloud
rostopic echo -n 1 /quad_0/imu
rostopic echo -n 1 /quad_0/lidar_slam/odom
```

- `/map_generator/global_cloud`：输入给仿真的全局三维 PCD 场景。
- `/quad0_pcl_render_node/cloud`：MID-360 模拟雷达的局部三维点云。
- `/quad_0/imu`：模拟 IMU。
- `/quad_0/lidar_slam/odom`：无人机里程计真值接口。

## 无图形界面运行

```bash
roslaunch test_interface single_drone_mid360.launch rviz:=false
```

## 多无人机示例

```bash
roslaunch test_interface triple_drone_mid360.launch
```

## 重新编译

```bash
catkin_make -DMARSIM_BUILD_GPU=OFF
```

GPU/OpenGL 渲染目标当前未编译，因为系统未安装 `libglfw3-dev`；官方启动文件默认
使用的 CPU 点云渲染器已完整编译并可运行。若以后安装该依赖，可将构建选项改为
`-DMARSIM_BUILD_GPU=ON`。
