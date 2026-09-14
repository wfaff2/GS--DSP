# 雷达感知与 Depth-CBF 仿真说明

本工作空间保存从原论文仿真工程迁出的 2026-09-01 之后新增内容。原论文仿真工程的恢复基线为 Git 提交 `9ca92a5`（2026-08-14，9 月 1 日前最后一次提交）。

## 工程组成

- `src/MARSIM`：已有的 MARSIM 点云渲染器。
- `src/coni_mpc`：迁移时的完整 ROS package，包含原三架无人机跟随一架无人车仿真，以及新增的 M1、M2、M2.5 雷达感知/Depth-CBF 代码。
- `src/no_catkin_compilation`：`coni_mpc` 使用的 ACADO 生成代码和辅助内容。
- `results/depth_cbf_m25`：迁移前已有的 M2.5 运行数据、日志和汇总结果。

## 当前雷达配置

当前 M1/M2 和 M2.5 启动文件均使用 MARSIM 自带的 CPU Livox Mid-360
模式：水平视场 360 度、垂直视场 90 度、量程 1--15 m、扫描频率
10 Hz、角度离散 0.2 度，`use_minicf_pattern=1`。雷达输出话题仍为
`/quad0_pcl_render_node/sensor_cloud`。

`results/depth_cbf_m25` 中迁移前的批量结果由旧 Livox Avia 配置生成，
仅作为历史结果保留。Mid-360 新结果单独写入
`results/depth_cbf_m25_mid360`；切换后必须重新运行完整 M2.5，不能把旧
结果标记为 Mid-360 验证结果。

原来的水平 FOV 边缘测试在 360 度雷达下已改为方位覆盖测试；当前批量
测试尚未增加俯仰运动，因此 90 度垂直 FOV 的边缘性能还没有被单独验证。

## 编译

```bash
cd /home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg coni_mpc -j2
catkin_make point_cloud_preprocessor_test depth_cbf_regressor_test -j2
```

不要直接执行整个 MARSIM 工作空间的 `catkin_make tests`：MARSIM 自带的
`uav_utils-test` 在当前环境中缺少 `-luav_utils` 链接目标，这与上述两组
Depth-CBF 测试无关。

## 单元测试

```bash
cd /home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws
./devel/lib/coni_mpc/point_cloud_preprocessor_test
./devel/lib/coni_mpc/depth_cbf_regressor_test
```

## M2.5 全部场景测试

```bash
cd /home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws
src/coni_mpc/scripts/run_depth_cbf_m25.sh "$(pwd)"
```

Mid-360 结果写入 `results/depth_cbf_m25_mid360`。M2.5 测试启动文件会发布测试用无人机/无人车里程计，因此不要和原三机跟车控制仿真同时启动。
