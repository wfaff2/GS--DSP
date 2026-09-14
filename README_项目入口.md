# 小样本学习的去中心集群协同跟踪：项目入口

本目录不只有 MARSIM。完整 ROS 工作空间位于 `marsim_ws`，项目根目录下
另外提供了几个直接入口，方便在文件管理器或 IDE 中查看：

- `激光雷达与Depth-CBF代码`：`coni_mpc` ROS package，包含原三架无人机
  跟随一架无人车仿真、Mid-360 点云接入、Depth-CBF、M1、M2 和 M2.5。
- `M1_M2_M2.5测试代码`：按阶段列出的测试源文件、M2.5 节点、启动文件、
  批量脚本和分析脚本入口。
- `M1_M2_M2.5测试结果`：M2.5 历史 Avia 结果和新的 Mid-360 测试结果。
- `MARSIM仿真器代码`：MARSIM 本体。
- `ACADO依赖代码`：`coni_mpc` 编译所需的 ACADO/qpoases 代码。

这些入口指向 `marsim_ws` 内的唯一源码，不是第二份副本，因此从任一入口
修改文件都会修改同一份代码，不会产生两个版本。

## 关键文件

- M1/M2 启动：`激光雷达与Depth-CBF代码/launch/marsim_depth_cbf_uav1.launch`
- M2.5 启动：`激光雷达与Depth-CBF代码/launch/depth_cbf_m25.launch`
- Mid-360/Depth-CBF 参数：`激光雷达与Depth-CBF代码/parameters/depth_cbf.yaml`
- 点云与 Depth-CBF 源码：`激光雷达与Depth-CBF代码/src/depth_cbf`
- 单元测试：`激光雷达与Depth-CBF代码/test`
- 批量测试脚本：`激光雷达与Depth-CBF代码/scripts/run_depth_cbf_m25.sh`

更详细的编译和测试命令见
`marsim_ws/README_雷达感知_Depth_CBF.md`。
