# 三维保守 Field-HOCBF：实现与验证记录

> 历史实现记录。2026-10-23 的活动算法已改为三维置信度加权、C2 紧支集安全场，
> 不再对所有正概率体素声称完整几何下界。当前公式、适用条件和回退计时见
> [当前实现说明](field_confidence_and_fallback_20261023.md)。下文保留原始版本记录。

本次基线为 `835cd7b39bb2d75d5b9f816e40c09c409a9641ab`，分支 `1016`。
目标是对 DSP 提供的占据表示构造三维几何保守场，并沿用现有 ACADO 仿射接口。
现有用户场景参数及证明文档保留；本次不提交 Git。

## 数据语义与认证对象

活动发布端是 `dynamic_occpuancy_map/src/map_sim_example.cpp` 的
`publishFutureOccupancy3D()`；活动地图头文件是 `include/dsp_dynamic.h`。

- `x/y/z` 来自 `getVoxelPositionFromIndexPublic()`，再加与输入点云关联的
  UAV 世界位置。它是该消息中世界坐标下的网格中心。
- 网格尺寸为 `66×66×40`，三轴边长均为 `VOXEL_RESOLUTION=0.15 m`。
  21 个预测切片覆盖 `0..2 s`，间隔 `0.1 s`。
- `occupancy` 是 DSP 粒子质量经 `occupancyProbabilityFromMass()` 映射并
  限制到 `[0,1]` 的值。发布集合明确为 `occupancy>0` 的全部单元。
  `getOccupancyMapWithFutureStatus(...,0.2,...)` 的阈值只筛选可视化点云，
  不筛选此预测消息。
- `vx/vy/vz` 是落入该目标体素的粒子质量加权平均世界速度。它不是网格中心
  的时间导数，本次不再用它移动网格中心或计算网格中心导数。
- 发布循环每个 `(voxel_index, stage_index)` 仅访问一次，没有 Top-K。
  消息未提供可靠整数索引、原点或分辨率；`source_id` 只是消息内点序号。
  控制端保留全部点，不进行近邻合并或跨快照去重。

认证对象是发布端正概率支持集在所选 stage、所选三维认证范围内的体素。
未发布单元没有 free/unknown 分类，不能由“未发布”推断真实空间自由。
上游观测、粒子容量、粒子删除及新生粒子预测规则都不提供无漏检保证。
有效空集合仅表示该表示中无已发布占据项。

## 实际数学模型

先在 nominal 位置周围按现有 `prune_radius=2 m` 选取三维中心距离满足
条件的全部体素，并冻结这个集合。对此固定集合，令

\[
q_m=p-c_m,\quad s_m=\sqrt{q_m^\top q_m+\varepsilon_d^2},\quad
r_m=\tfrac12\sqrt{\ell_x^2+\ell_y^2+\ell_z^2},
\]
\[
d_m^{\rm cert}=s_m-\varepsilon_d-r_{\rm UAV}-r_m,\qquad
\kappa_m=1+\alpha P_m\ge1,
\]
\[
h_F=d_{\min}-\sigma\log\sum_m\kappa_m
e^{-(d_m^{\rm cert}-d_{\min})/\sigma}-d_{\rm margin}.
\]

当前实际体素为立方体，故 `r_m=√3·0.15/2≈0.129904 m`。使用原有
`uav_radius`；新增 `field_distance_margin=0.20 m` 明确表示额外表面间隔，
不再叠加旧 `field_d_safe=0.48 m`。这个显式实验参数不代表经过动力学认证。
默认 `alpha=0`，因此所有纳入单元的基础权重均为 1。

因为 `sqrt(x²+eps²)-eps <= x` 且最小距离项的权重至少为 1，有

\[
d_m^{\rm cert}\le\|p-c_m\|-r_{\rm UAV}-r_m,\qquad
h_F\le\min_m d_m^{\rm cert}-d_{\rm margin}.
\]

外接球包含整个体素，所以 `h_F>=0` 对上述选定体素集合给出 UAV 球与
体素的几何间隔下界。这个代数结论不依赖 occupancy 的概率校准。

梯度、Hessian 及冻结权重/运动学模型下的时间导数为

\[
n_m=q_m/s_m,\quad H_m=I/s_m-q_mq_m^\top/s_m^3,\quad
w_m=\frac{\kappa_m e^{-(d_m^{\rm cert}-d_{\min})/\sigma}}
{\sum_j\kappa_j e^{-(d_j^{\rm cert}-d_{\min})/\sigma}},
\]
\[
g=\sum_mw_m n_m,\quad
\nabla^2 h_F=\sum_mw_mH_m-sigma^{-1}
\left(\sum_mw_mn_mn_m^\top-gg^\top\right),
\]
\[
v_{r,m}=v-v_m,\quad \dot d_m=n_m^\top v_{r,m},\quad
\ddot d_m^0=v_{r,m}^\top H_m v_{r,m}
+n_m^\top(a_{\rm ni}-\Lambda v-a_m),
\]
\[
\dot h_F=\sum_mw_m\dot d_m,\quad
L_f^2h_F=\sum_mw_m\ddot d_m^0-sigma^{-1}
\sum_mw_m(\dot d_m-\dot h_F)^2,
\]
\[
A=\Lambda^\top g,\qquad
b=-[L_f^2h_F+(\gamma_1+\gamma_2)\dot h_F+\gamma_1\gamma_2h_F].
\]

世界中心在每个选定 stage 内冻结：惯性模式 `v_m=a_m=0`；非惯性模式

\[
c_N=R_{NW}(c_W-p_g),\quad v_m=-R_{NW}v_g-\omega\times c_N,
\]
\[
a_m=-a_{g,N}-2\omega\times v_m-\beta\times c_N
-\omega\times(\omega\times c_N).
\]

上述非惯性运输项与 UAV 模型一致，未删除 Coriolis、Euler 或向心项。
这些是冻结网格空间场的计算模型导数，不是实际随时间更新的 DSP 场的完整导数。

ACADO 继续接收每 stage 六个 profile 数值。其现有状态反馈形式为

\[
A^\top(u+k_v\odot v+k_p\odot p)+\delta-b_{\rm profile}\ge0,
\quad b_{\rm profile}=b+A^\top(k_v\odot v_{\rm nom}+k_p\odot p_{\rm nom}).
\]

在 nominal 点它恢复 `A·u+delta-b>=0`；离开 nominal 后仍是现有仿射代理，
没有新增非线性 Taylor 余项界。ACADO 维度、输入、预测时域及 Route A 回退不变。

## 光滑性与保证边界

对非空固定有限集合、`eps>0`、`sigma>0`，上述场在空间上光滑，包含体素
中心处。shifted LSE 中的 `d_min` 仅用于稳定计算，不单独微分该分支。

硬半径筛选随 nominal 变化会切换集合。新项加入时会引起负的 log-sum 跳变，
所以动态 `prune(p)` 与场评估组成的全局映射不能声称空间连续。
本次保证限定于每个冻结认证集合；`prune_radius` 未获动态可达/制动证明。
范围外体素、stage/快照切换、感知失效、离散执行、slack 与旧计划回退都不在
这个几何不等式的闭环保证内。没有跨帧时间平滑或鲁棒前向不变性证明。

## 修改位置与六项结果

| 文件 | 具体位置/作用 |
|---|---|
| `include/coni_mpc/field_hocbf.h` | `Config`、`validConfig`、`prune`、`computeConstraint`：三维集合、证书距离、LSE 和全部导数 |
| `src/coni_mpc/num_sim_mpc.cpp` | 构造参数、`fieldOccupancyCallback`、`run()` 内 `buildFieldProfile`：严格快照接收、固定中心、非惯性变换、状态与耗时 |
| `include/coni_mpc/num_sim_mpc.h` | `FieldHocbfStageDebug`、快照状态与 timeout 成员 |
| `src/num_sim/num_sim_non_one_point_node.cpp` | `writeFieldHocbfHeader/Rows`：每 stage 状态、构造耗时及 MPC 时间列 |
| `parameters/num_sim_non_one_point.yaml` | `cbf/field_*` 新几何配置；原有 `a_max_xy=6`、`num_uavs=1` 等用户修改保留 |
| `include/coni_mpc/field_replay_diagnostics.h` | v2 配置/导数/计数序列化及固定中心 replay，拒绝 v1 |
| `test/field_hocbf_test.cpp` | 八项几何、集合、梯度/Hessian、Lie 导数与 Z 约束测试 |
| `test/field_replay_diagnostics_test.cpp` | 五项回放、版本、固定世界中心与非惯性有限差分测试 |
| `test/field_replay_tool.cpp`、`test/field_replay_event_tool.cpp` | 更新配置与固定中心重建 |
| `test/field_snapshot_integration_test.cpp`、`test/field_snapshot_integration.test` | 实际 NumSim 接收、状态与 ACADO 集成验证 |
| `CMakeLists.txt` | 注册新增 rostest；既有 rostest 依赖可复用 |
| `TASK_LOG.md`、本文 | 跨会话工作记录、模型与证据边界 |

| 修改项 | 结果 |
|---|---|
| 1. 取消 max35 | 完成。认证范围内 eligible 与 selected 完全一致 |
| 2. 取消二维 separation | 完成。缺少可靠栅格索引，因此不去重，所有相邻/重复输入保留 |
| 3. 删除 min_occupancy 二次门限 | 完成。纳入全部发布的正 occupancy；`alpha=0`，`kappa=1` |
| 4. 删除竖直切片 | 完成。以三维中心距离与现有 2 m 半径筛选 |
| 5. 非归一化保守 shifted LSE | 完成。未除权重和/体素数；验证下界关系 |
| 6. 三维几何与导数 | 完成。三个坐标轴及全部非惯性运输项保留 |

完整集合的条件是“发布 stage 在当前认证范围内的集合”。对全部发布体素
的全局保证、动态硬筛选映射的全局空间光滑性均不在上述完成结论中。
若把这两项也作为验收要求，则在保留当前硬筛选的条件下应标为 **BLOCKED**。

## 数据有效性与诊断

callback 在读取点之前验证字段类型、偏移及紧凑 row layout，空消息也进行
schema 校验。错误 frame、非法行/类型、越界、截断等拒绝整条消息，保留
旧快照并记录 `invalid`；不会把坏消息解析为新空集合。带 padding 的
organized cloud 当前明确拒绝，实际 DSP 的紧凑布局通过集成验证。

旧有效快照在配置 timeout 内仍可继续使用；`invalid` 表示最新输入被拒绝，
并不表示旧快照被重新认证。超过 `field_timeout=0.3 s` 后不再启用该 Field。
`missing`、`stale`、`invalid` 与有效认证区域的 `valid_empty/valid_nonempty`
在每 stage CSV 中区分。数据失效后的 Field 禁用不构成无障碍空间证书，
本次未新增失效安全控制器。

`field_selected_rel_v*` 与相对速度最大值保留为发布的 **particle velocity
proxy** 诊断；它们与约束使用的 fixed-center 运动学不同。原始发布速度也
保存在快照回放记录中。`field_active` 表示该 stage 启用了 Field 行，
不等同于 QP 不等式恰好取等号。

## 验证结果（最终源码）

- `coni_mpc` 包构建通过。8 项 Field 单测、5 项 replay 单测和 1 项 ROS
  集成测试全部通过；回放工具 self-test 及 event 工具编译通过。
- 随机三维/多分辨率证书与体素表面间隔比较通过；覆盖单体素、密集集合、
  超过35项、重复/相邻、低 occupancy、上下障碍、中心及近中心。
  梯度、Hessian、一/二阶导数、A/b 及任意轴旋转/平移非惯性变换均通过有限差分。
- ROS 集成测试确认 52 个低 occupancy 项完整入场；把发布速度从
  `+1000` 改到 `-1000 m/s` 不会改变冻结网格场的 stage-0 值和导数。
  同时验证 missing、有效空、非法 frame/行、过期、短 point_step 以及空消息
  的非法 datatype/offset 都有正确状态。
- 原有 ACADO 契约保持 `N=20,NX=7,NU=7,NOD=99,NPAC=12`，Field profile
  为 `6×21`，未引入优化变量或重新生成代码。两种 frame 的实际 DSP 仿真通过。

两次最终仿真各 3 s、单 UAV，使用当前 scene/YAML，障碍物通过实际
MARSIM/LiDAR/DSP 链进入，没有调节物理限幅或 solver 阈值以取得成功结果。

| 指标 | noninertial | inertial |
|---|---:|---:|
| 控制调用/失败 | 150 / 0 | 150 / 0 |
| 最大候选数/stage | 1189 | 1169 |
| 最大认证并保留数/stage | 335 | 399 |
| 保留数量超过35的 stage | 226 | 208 |
| 启用 Field 的 stage | 328 | 303 |
| 每周期21个stage构造总耗时 P95/最大 ms | 0.891 / 1.241 | 0.782 / 1.233 |
| MPC feedback P95/最大 ms | 1.510 / 2.074 | 2.153 / 2.982 |
| MPC preparation+feedback 最大 ms | 2.682 | 3.577 |
| 实际调度周期平均/最大 ms | 3.763 / 7.060 | 3.935 / 8.009 |
| 20 ms 调度超时 | 0 | 0 |
| 有效 stage 回放/错误 | 2961 / 0 | 2982 / 0 |

启动时分别有9和8次控制尚无 DSP 快照，均记录为 missing。有效空区域保持
单独记录。上述短样本不代表全工况时限、避障成功率或连续时间安全证明。
现有 `total_pipeline_time_ms` 在 MPC 模式下只反映其既有 core 时间口径；
表中完整调度周期使用日志中的 scheduler 墙钟统计，没有把该列冒充完整链耗时。

独立 O3 基准（每规模10次、每次21个stage，含 selection/copy/Field，不含
ROS、坐标转换、CSV 或 MPC）平均为：35项0.027 ms、100项0.070 ms、
1000项1.030 ms、10000项13.889 ms/预测时域。新增计算与体素数增长有关；
大规模和多 UAV 时可能消耗20 ms预算，当前没有最坏时限保证，也未恢复有损截断。

原始配置快照、命令、CSV、日志、版本化 replay、测试 XML、性能脚本和
源码/二进制哈希位于 `audit_reports/field_3d_20261022_144435/`。
最初集成记录保存在 `initial_*`；最终记录在 `noninertial/`、`inertial/`。
用户原有三份未跟踪证明/设计文档、场景 YAML 及生成代码的保留哈希检查通过。
