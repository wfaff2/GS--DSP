# DSP 体素池化与 ACADO 平滑开关势垒模块

本次提供两个集成在现有工程中的模块，以及模型生成入口。当前仿真仍使用
原有外部仿射 Field-HOCBF；没有把新布局写入旧求解器。

## ROS 类成员调用

`NumSimMpc::buildVoxelOnlineData(pos, raw_voxels)` 复用
`field_hocbf::KinematicPoints`，内部调用 `VoxelOnlineDataPool::build()`。
每个节点实例从自己的私有 `cbf/voxel_field` 命名空间加载
`pool_leaf_size`、`n_obs`、`padding_distance`。未配置时调用接口会
抛出明确异常；参数无效时拒绝初始化，不静默退回硬编码值。

```cpp
// 已有的 DSP 单个预测切片，经现有流程变换到求解器坐标系。
// raw_voxels[i].position 与 pos 必须属于同一坐标系。
const std::vector<double> online_data = buildVoxelOnlineData(pos, raw_voxels);
```

不要将 DSP 的不同 `stage_index` 混合池化。非惯性模式下，应先按该阶段的
UGV 位姿变换体素位置，再使用同一坐标系的查询位置。接口不会猜测 frame。

算法顺序：3D `floor(position / pool_leaf_size)` 哈希网格 → 无权质心与占据率
相加封顶至 1 → 按 3D 欧氏距离排序 → 保留最近 `n_obs` 个并保持其真实
占据率 → 用有限远点、零占据率补齐。池化类不依赖 `r_cut`，不执行距离掩码。
结果严格为 `4*n_obs` 个 double，顺序是 `[px, py, pz, P_occ]`。
相同距离用网格键排序，避免 unordered_map 遍历顺序影响输出。
非有限坐标/占据率、非正占据率及不可表示的网格索引被跳过。

输出、网格桶、单元与排序数组在循环前预分配容量；unordered_map 每个新
网格仍需要节点分配。没有在每个原始点处重新分配 vector。

## ACADO 装配与生成

`acado_model/voxel_barrier_model.h` 中的类拥有 `4*n_obs` 个 OnlineData。
`assemble(ocp, x, y, z)` 累加 Wendland 多项式乘 tanh 平滑开关、占据率与指数项，
返回 `d_eval` 并添加 `d_eval - d_safe_robust >= 0` 路径约束。
`n_obs` 在**生成时**确定；改 YAML 数量不会自动改变已编译求解器的维度。

模型入口接受外部参数，参数顺序如下；值应来自 YAML 中的 `voxel_field`：

```text
quadrotor_model_codegen --voxel-field n_obs r_cut sigma d_safe_robust \
    distance_epsilon partition_floor k_steep --output isolated_directory
```

不带参数维持原有生成行为。新模式要求独立导出目录。新增块追加在现有
87 个 OnlineData 之后，因此 `n_obs=60` 时 `ACADO_NOD=327`，每个阶段的
体素槽范围为 `[87, 327)`，并新增一个路径约束。本次验证的独立导出目录
是 `/tmp/dsp_voxel_smooth_codegen_validation`。

后续接入实时求解器时，必须同步 wrapper 的 NOD/路径约束布局、边界及
初始化，并为每个 shooting stage 写入该阶段的池化数据；包括终端 OnlineData。
当前 `MpcWrapper::setFieldHocbf()` 接收的仍是原有仿射数据，不能传入此数组。

## 数值保护与连续性边界

请求中的原公式在 `S=0` 时对数无定义；`sqrt(||delta||²)` 在零距离处的
自动微分也会奇异。本实现使用两个明确配置的数值保护：

```text
d = sqrt(||delta||² + distance_epsilon²)
d_eval = -sigma * log(S + partition_floor)
```

这两项改变了原公式，不能称为完全等价。正 epsilon 平滑径向指数在体素中心
的尖点；正 floor 保证空场的约束值有限。要求
`partition_floor < exp(-d_safe_robust/sigma)`，使空场安全约束可满足。
floor 降低 `d_eval`；epsilon 增大距离，两者对障碍附近约束的影响不同。

平滑开关为 `0.5 + 0.5*tanh(k_steep*(r_cut-d))`；`k_steep` 在 YAML 中配置为
20.0 并传入模型成员。当前 ACADO 没有符号 tanh，采用数学等价表达式：

```text
z = k_steep*(r_cut-d)
smooth_switch = exp(2*z)/(1+exp(2*z))
```

该表达式避免 `0.5+0.5*tanh(z)` 在远处相减消差。因为 `d>=0`，正指数
上界为 `2*k_steep*r_cut`，构造时要求它小于 `log(DBL_MAX)`；负指数下溢时
开关为零，不产生正指数溢出。

连续性与预测边界：

- 开关随优化后的预测位置重新计算，选中的远处体素保持真实概率；预测位置
  接近时无需外部重新激活。正则化距离与开关的复合场对固定 OnlineData 平滑。
- tanh 开关是渐近衰减而非严格紧支撑；多项式在 `d=r_cut` 为零，其外侧
  仍有衰减尾部。它不保证场对距离全局单调。
- 体素中心仍是 OnlineData；动态障碍自身的运动须由调用方逐阶段更新位置，
  本次开关不会自动预测障碍速度。
- 若支撑域内超级体素数量超过 `n_obs`，Top-N 删除仍有非零贡献的体素，
  候选交换仍可改变场及其导数。空间池化降低数量，但本身不保证容量足够。
- 原始概率图更新、池化质心变化和预测切片切换也可能造成跨控制周期变化。
- 该约束是预测 shooting node 上的位置安全约束，没有新增 HOCBF 李导数行，
  也不能单凭生成与编译成功宣称连续时间或真实障碍表面安全。

真正切换控制链前仍需验证候选容量、逐阶段位置和运行表现。本次没有替换
当前控制链，也没有宣称已验证的避障安全结果。
