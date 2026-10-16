# UGV Yaw-only 角速度噪声修改的闭环验证（2026-10-12）

## 结论与验收状态

**噪声模型校正通过，高度杠杆臂放大机制已经消除。** 修改后 1500 个周期的 UGV 真值/测量角速度 X/Y 都严格为 0；UGV roll/pitch 姿态严格为 0；实际 MPC OnlineData 的全部 21 个 stage 也没有 X/Y 角速度。原来约 2 m 高度乘以 Roll/Pitch 角速度噪声的前馈项逐周期严格为 0。Yaw 角速度噪声以及三轴线速度噪声保留，和本次修改前版本重跑的非目标噪声样本逐 double 完全一致。

**常态补偿和命令波动减少，但整体控制性能没有无条件通过验收。** 相对本次 before，补偿变化 P95 从 0.377977 降到 0.183218 m/s，世界指令变化 P95 从 0.510951 降到 0.313229 m/s，超过 0.5 m/s 的次数从 84 降至 14。但 after 的最大世界指令跳变为 2.053921 m/s，超过 before 的 1.794766；after 有 13 次不可行 QP/回退，before 有 2 次；调度器超时为 21 次与 11 次。after 的真值 tracking RMS 为 0.388347 m，略高于 before 的 0.367770 m。

**剩余最大跳变首先出现在 MPC 返回值，发生在不可行求解后的恢复周期。** 它不是 Roll/Pitch 高度前馈重新出现。保留的 yaw 噪声仍经平面位置杠杆臂造成较小补偿波动，线速度测量噪声也继续进入 world mapping。

**物理避障只能部分核验。** 两次均记录 collision=0；动态 box 的 ground-truth 状态和尺寸支持表面间隙重建，after 最小约 0.332191 m。静态场景只有 PCD，缺少连续实体几何定义，因此静态真实表面间隙不可获得，不能用点距离或 collision=0 宣称完整闭环安全。

建议**结束“平面 UGV 被注入 Roll/Pitch 角速度噪声”这个模型问题的排查**。剩余不可行 QP、当前约束下的回退有效性、恢复周期大幅输出及实时性应作为独立控制/工程问题保留。本次没有开始下一阶段算法设计，也没有调参、过滤或改变映射。

## 1. 实验身份、配置和对照边界

独立归档：

[/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi)

- historical：历史 /tmp/dsp_field_provenance_complete_20261012，作为历史非配对对照。
- after：本次当前 yaw-only 版本的完整 DSP + LiDAR 闭环运行。
- before：本次在独立目录重新编译历史 NumSimMpc 源码、重新链接的修改前版本；没有撤销当前工作树，没有改原有库/对象/节点输出。
- before 与 after 顺序运行，各 30 s、1500 周期、逻辑控制周期 0.02 s，state/provenance 采样为 0–29.98 s。MPC 20 个控制 stage，预测步长 0.1 s，共 2 s，含 21 个状态/OD stage。
- 集成 launch 为单 UAV、UGV 圆轨迹 R=7 m，reference N=[0,1.5,2]，noninertial、frozen rollout，原有三个静态方棱柱、L 墙和两个移动 box，Field-HOCBF、DSP 未来占据均开启。
- YAML 的 num_uavs=4、figure_eight 是普通 benchmark 默认值；dsp_mpc_sim.launch 已有明确覆盖，不是本次修改配置制造比较结果。
- 版本 HEAD：1e66bf5017cd93d07c2fa5a3c67b5cc2ab76f27f，已有 dirty 工作树保留。initial_worktree.diff、initial_git_status.txt、各 run/source/ 与 experiment.json 已保存。
- before 二进制 SHA256：f9831a76ea793078233465fd8bd0050a0d0771c0b5f6420bfa719c08ff7701c8。
- after 二进制 SHA256：dd576ff47ead4abb08ca37f74da4d4ee9019b9d273ed9f5f8393a27a8859d932。

全部生效 runtime 参数已展开比较，见 [runtime_full_parameter_differences.json](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/runtime_full_parameter_differences.json)。历史与 after 仅输出目录、ROS run_id/URI 不同；before 另有静态地图路径差异，但它是隔离目录中指向同一文件的 symlink，地图内容相同。控制/场景/感知参数数值完全一致，未调整任何权重、增益、安全半径、slack 或噪声标准差。

### 对照不能称为严格配对闭环实验

独立构建可行并已执行：baseline_build/build_before.py 使用历史 NumSimMpc.cpp，单独编译该对象，替换独立静态库的相应 member，并将同一节点对象/其余库链接到独立二进制。baseline_build/package/ 的 launch-prefix 只选择该二进制；不改变控制逻辑。编译命令、实际启动二进制和 hash 已归档。它是隔离重编译/链接，未声称干净重建全部 ROS 依赖。

UGV、控制器、障碍配置/初态、控制噪声 seed/std 相同；非目标 UGV yaw 与线速度噪声序列也实测完全相同。但 DSP 无法严格固定全部随机输入：

- 实际 map_sim_example.cpp 引用 dsp_dynamic.h。
- [dsp_dynamic.h:712](/home/jjm/桌面/DSP/marsim_ws/src/dynamic_occpuancy_map/include/dsp_dynamic.h:712) 调用 srand(time(0))；[生成高斯缓冲:1309](/home/jjm/桌面/DSP/marsim_ws/src/dynamic_occpuancy_map/include/dsp_dynamic.h:1309) 使用 default_random_engine(time(NULL))。DSP 实际运行种子没有单独归档，不能编造其值。
- LiDAR/DSP 的墙钟异步调度、动态 box 的墙钟起动时刻和占据 profile 不是逐样本相同。
- 本次没有为固定 DSP 种子修改感知算法，也没有将历史 bag 强行输入不同闭环轨迹冒充严格配对。

因此 before/after 是**同设置、同非目标控制噪声的版本对照，感知输入非严格配对**；historical/after 是历史非配对对照。可以证明噪声屏蔽与高度项消除，描述这些运行的性能差异；不能将全部 tracking、failure、最大跳变差异归为单一噪声修改的严格总因果效果。同为 26.28 s 也不代表同一物理事件。

## 2. 源码与模型一致性检查

### 2.1 实际修改范围

与历史运行的源码 hash 比较，运行时控制源码的差异仅为 [num_sim_mpc.cpp:994](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/coni_mpc/num_sim_mpc.cpp:994)：

1. 仍以原标准差抽取三个角速度高斯样本。
2. 将准备注入的 noise.x、noise.y 置为 0。
3. 将 noise 加到原始 UGV 角速度。

保留三次抽样是为了保持后续随机序列。没有将真实 X/Y 角速度强制清零；此平面 UGV 的真实 X/Y 本来就为零。噪声模型因此为 (0,0,ωz+nz)。

Yaw std=0.0435 rad/s、UGV 线速度各轴 std=0.025 m/s、noise_seed=1（quad_id 依原规则偏移）均未变。位置/相对速度/姿态/IMU 噪声参数未变，线速度噪声仍在原位置生成。YAML 仅增加“yaw rate only”的注释。没有意外改变控制权重、约束、warm start、预测阶段或周期。记录见 source_audit.json。

### 2.2 真值与模块使用路径

|环节|源码证据及结果|
|---|---|
|UGV 真值圆轨迹|[node.cpp:5188](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:5188) 只用绕Z四元数，point(13/14)=0，point(15)=yaw rate|
|轨迹插值/终点外推|node.cpp:3806、3854：yaw-only quaternion slerp/Z轴外推，不能产生虚假ωxy|
|frozen prediction|node.cpp:2209、2268：各stage纯yaw姿态和yaw-only非惯性profile|
|测量与 MPC|num_sim_mpc.cpp:994、1048、1065：测量ω进入state_estimate，stage0由current-time估计覆盖；其余stage使用原预测。mpc_wrapper.cpp:379 逐stage直写OD|
|Field-HOCBF|num_sim_mpc.cpp:2029、2081、2123、2207：同一omega/beta/a profile参与world→N和漂移/affine profile计算，没有独立再加角噪声|
|最终世界指令|[node.cpp:3305](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:3305)：从同一控制器取得测量角速度、原始UGV姿态/位置，执行原有公式|

Stage0 为带 yaw 噪声的估计，stage1–20 是 frozen rollout 的无角噪 yaw-only profile；不同阶段不要求相同，但 X/Y 均为零。原有实现把 odometry angular 直接用作 omega_non，在该场景纯yaw不变量下 W 与 N 的值相同；本验证不推广为一般三维传感器坐标约定正确性的证明。

## 3. 运行时三轴角速度与高度杠杆项

after 1500 周期：

- max |ωx_true| = max |ωy_true| = 0。
- max |ωx_measured| = max |ωy_measured| = 0。
- 从实际 UGV rotation matrix 提取的 max |roll|、max |pitch| = 0 rad。
- 1500×21 stage 的 QP OnlineData 中 max |ωx/y| = 0；stage0 与 provenance profile/mapping 完全一致，x0 与相应估计状态完全一致。
- Yaw noise 平均 −0.000617927 rad/s，std=0.043727331 rad/s，RMS=0.043731696 rad/s，1500 个样本全部非零，与原设定 σ=0.0435 基本一致。
- before/after 的 yaw noise 及三轴 UGV 线速度 noise 全部逐 double 相同，最大差为 0。线速度噪声没有被删除。
- 最终世界映射的 ω×p 的 Z 分量严格为 0。

原公式保持：

ω×p = (−ωz py, ωz px, 0)

定义 Roll/Pitch 噪声的高度前馈项：

c_height_W = R_WN (ny pz, −nx pz, 0)

修改前 before 的该项相邻周期变化 mean/P95/max 为 0.158464/0.302722/0.512539 m/s；after 三个值严格为 0。更一般地，∂(ω×p)/∂pz=(ωy,−ωx,0)=0，所以“约2m高度 × 人工Roll/Pitch角速度误差”的直接放大机制已消除，不依赖当前 tracking 轨迹是否相同。

Yaw 噪声项仍为 R(−nz py,nz px,0)，只依赖平面杠杆臂。不能期待去掉 RP 后整个世界前馈变成无噪，也没有修改 yaw/线速噪声或 UAV 状态估计噪声。

## 4. 指令突变、剩余事件与动力学响应

所有相邻周期统计用 1499 对样本，Δv=||v_cmd[k]−v_cmd[k−1]||₂，单位 m/s，未将它称为实际加速度。

|指标|历史修改前|本次修改前重跑 before|修改后 after|
|---|---:|---:|---:|
|世界指令 Δv 最大值 (m/s)|0.866241|1.794766|2.053921|
|世界指令 Δv P95 (m/s)|0.505089|0.510951|0.313229|
|世界指令 Δv 均值 (m/s)|0.261517|0.271419|0.146774|
|Δv > 0.5 m/s 次数|80|84|14|
|Δv > 1.0 m/s 次数|0|4|2|
|ω×p 变化最大值 (m/s)|0.604779|0.609341|0.360026|
|ω×p 变化 P95 (m/s)|0.363193|0.377977|0.183218|
|ω×p 变化均值 (m/s)|0.192039|0.196752|0.072840|
|MPC 返回速度变化最大值 (m/s)|1.078647|1.860762|1.920928|
|MPC 返回速度变化 P95 (m/s)|0.304445|0.279938|0.221273|
|MPC 返回速度变化均值 (m/s)|0.102638|0.102494|0.079578|


相对 before，after 的世界变化均值下降约 45.9%，P95 下降约 38.7%，>0.5 m/s 次数下降约 83.3%；补偿 P95 下降约 51.5%。它们是本次运行的描述性比较，不是严格配对因果估计。最大世界变化反而上升，不能只挑 P95 宣称全部性能提升。

### 4.1 最大事件的逐环节定位

|当前周期|上一周期→当前求解状态|世界 Δv|MPC 返回 Δu|补偿项 Δ(ω×p)|Δ限幅修正 / Δ最终后处理|
|---|---|---:|---:|---:|---:|
|327，6.54 s|−2失败回退 → 0成功|1.945110|1.920928|0.074119|0 / 0|
|336，6.72 s|−2失败回退 → 0成功|2.053921|1.897496|0.177930|0 / 0|

两个最大事件的大幅值先出现在实际 MPC 返回速度；不应把失败 QP 中的 partial primal、expand 数值当作本周期执行命令。恢复周期成功解在当前 Field 行上有绑定：cycle327 为1个 horizon stage，cycle336 为3个；第0阶段仿射残差约浮点0。可以确认失败→回退→恢复输出切换与此处大变化同周期出现，尚未定位究竟哪些约束/RTI 输入导致不可行，不能仅凭 h<0 断言具体根因。

分解采用严格有序向量恒等式：

Δv = Δvg + RaΔu + Ra(Δω×pa) + Ra(ωb×Δp)
     + (Rb−Ra)(ub+ωb×pb) + Δ限幅修正 + Δ后处理

顺序固定、耦合分配不唯一；向量可以相加，贡献范数不能相加。全部三次运行闭合误差在浮点精度范围。after_top5_command_events.csv 保留完整贡献向量。

仍有世界变化大于小幅相对 MPC 输出的周期。例如 after 26.28 s：Δu=0.022392 m/s、补偿变化=0.128239 m/s、世界变化=0.218817 m/s，RP/高度项仍为0。这是保留的 yaw/线速噪声和映射耦合，不是旧高度机制复发；不将该新周期认定为历史恰好0.64571事件。

### 4.2 真值 tracking 与实际响应

UAV 真值来自 sensors.bag 的 /coni_mpc/quad_odom0。其 header.stamp 与 provenance 的 state_stamp 每行完全相同，world position 逐项一致；不是异步最近时间拼接。

参考 world position = p_UGV_W + R_WN [0,1.5,2]。真实 tracking 用真实世界位置与此参考计算，并与 rot_load_position_non−reference_N 独立交叉检查，最大差小于7×10⁻⁷ m（metrics输出精度）。metrics_steps 的 position/velocity 是带噪 N 状态，tracking_error 是带噪跟踪误差，另外输出，未当 world 真值。

加速度分别报告：

- 模型应用瞬间 t_k⁺ 的 a=(u_applied[k]−v_true[k])/tau，源于 [System.h:148](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/include/num_sim/System.h:148) 的实际模拟动力学，tau=(0.2,0.2,0.2)s。
- 真实速度的周期平均加速度 (v_true[k]−v_true[k−1])/0.02。分母用逻辑仿真积分步长，不用带墙钟调度抖动的ROS时间差。
- 下一周期真值速度与 u[k]+(v[k]−u[k])exp(−0.02/tau) 的最大误差 <7.82×10⁻⁹ m/s，支持状态/命令时序对齐。模型推导与差分量分别标记，都不是命令差当加速度。

|指标|历史修改前|本次修改前重跑 before|修改后 after|
|---|---:|---:|---:|
|真值跟踪 RMS (m)|0.557562|0.367770|0.388347|
|真值跟踪误差 P95 (m)|1.501881|0.820524|0.923763|
|最大真值跟踪误差 (m)|1.992237|1.262802|1.358712|
|实际水平速度最大值 (m/s)|1.493249|1.378567|1.419951|
|模型瞬时加速度最大值，三维范数 (m/s²)|5.323235|7.148032|7.957879|
|真实速度差分平均加速度最大值 (m/s²)|5.065728|6.802252|7.572922|
|动态 box 重建表面间隙最小值 (m)|0.146495|0.362690|0.332191|
|动态 box 零阶保持间隙最小值 (m)|0.151048|0.363650|0.333768|
|静态连续实体真实表面间隙|不可获得|不可获得|不可获得|
|记录碰撞标志|0|0|0|


水平模型瞬时加速度最大值：historical 5.315946、before 7.139393、after 7.953819 m/s²；水平 P95 为 2.072966、2.061082、1.772101 m/s²。after 三维最大约7.957879，周期平均最大约7.572922。模型瞬时值不冒充直接加速度传感器测量；可以从真实模拟状态和已验证动力学推导。

after 的平均/P95 波动、平均响应减小，但跟踪 RMS、最坏命令与最大加速度相对本次 before 并未更好；对历史 tracking RMS 又更好。这种参照差异也是不得把所有变化归因于修改的证据。

## 5. 避障几何、Field 约束及回退

### 5.1 真实物理间隙的可得性

动态：两个 box 的真实中心、速度、size 来自 /quad0_pcl_render_node/dynamic_obstacle_states，geometry_type=box，frame=world；map↔world 在 launch 中为 identity。确定性尺寸和速度与 runtime YAML 一致，size 在float精度范围内稳定。源码 [renderer:936](/home/jjm/桌面/DSP/marsim_ws/src/MARSIM/local_sensing/src/pointcloud_render_node.cpp:936)、:493、:546 描述配置、墙钟更新与真实状态发布。

以最近过去的真实消息按记录的恒定速度推进到 UAV odometry 的ROS时间，计算三维AABB signed distance：

δ=|p−center|−size/2
sdf=||max(δ,0)||₂ + min(max_i δ_i,0)
clearance=sdf−0.15

它使用实体尺寸和中心，不是 voxel中心、soft-min或 LiDAR点距离。所有1500周期都有两个box的有效来源；限最多0.2s推进，after最大消息年龄约0.1242s，before约0.1450s。必须使用ROS时间重建，不能以sim_time从YAML初始值外推，因为box在3s仿真等待期间已移动。

动态重建仍有精度边界：renderer消息stamp在位置更新前取得，且10Hz才更新/发布状态，CV推进是源码运动模型对记录状态的重建，非高频直接测量。另输出保持最近位置的ZOH对照；after两种clearance的全程最大差约0.0691m，before约0.0799m，消息年龄×速度的最大位移分别约0.0745/0.0870m。未据此声称连续时间无碰撞证明。两个模型的采样间隙均为正，after最小CV/ZOH为0.332191/0.333770m；before为0.362690/0.363650m。

静态：唯一权威记录是 A2_three_square_prisms_l_wall.pcd，没有生成此地图的连续solid/mesh元数据。虽然能从点块推得三个棱柱及两段L墙的范围，但将其拟合成实体会新增几何假设。因此本次**静态真实表面间隙及全环境最小真实间隙为不可获得**，未用PCD点距离替代。

metrics 的 truth_min_planar_surface_clearance 为NaN，因为它只遍历单机内部解析障碍/其他UAV；本launch obstacle_count=0、use_dynamic_obs=false，使障碍完全经 MARSIM→LiDAR→DSP 输入。该NaN不意味着无障碍或无限安全。collision=0仅是现有点采样碰撞监视器的记录，不能完成静态solid独立验收。

### 5.2 Field-HOCBF 原始 h、仿射约束、slack

|指标|历史修改前|本次修改前重跑 before|修改后 after|
|---|---:|---:|---:|
|第0阶段 Field profile 启用周期数|1135|1140|1140|
|全stage profile 启用行数，21 stage/周期|23975|24265|24305|
|全stage 原始 h 最小值|-0.247220|-0.287096|-0.255671|
|第0阶段原始 h 最小值|-0.197946|-0.171557|-0.255671|
|成功 QP 中 Field dual 绑定行数|4398|3935|3893|
|成功 QP Field 下界余量最小值|-2.6166e-12|-2.6859e-12|-2.5648e-12|
|成功周期返回值的第0阶段仿射残差最小值|-1.8687e-12|-2.2347e-12|-2.1814e-12|
|失败回退周期的仿射残差最小值|无失败周期|-6.367978|-11.640847|


h 是风险场/名义状态上的原始安全量，不能等同物理表面间隙，也不能仅凭仿射残差≥0声称 h≥0。全部 h 最小值为负已如实保留。

field_active=1表示profile启用；QP绑定按成功求解的 Field rows 9+10×stage、|dual|>10⁻⁸判断。每周期只有20个path约束stage，最后状态stage20不是额外终端path行。metrics 中 legacy solver_cbf_active/track_only 标志没有完整表达Field状态，不能拿“legacy未激活”推断Field未生效。

成功 QP 的Field lower slack与返回值第0阶段affine residual最低约−2.6×10⁻¹²，在浮点精度范围内；没有>10⁻⁸的Field下界违约。所有执行slack为0。失败QP的partial primal/dual未进入成功绑定/最优性结论。

after 13次失败均使用 previous-valid command。按**当前**第0阶段捕获的 A、b、x0评估实际返回值：

r=Aᵀ[u_return+((alpha1+alpha2)tau−1)⊙v_est+alpha1 alpha2 tau⊙p_est]+slack−b

after失败周期r全部为负，最小−11.640847、最大−0.495357；before的2次失败同样为负，最小−6.367978。该值是当前估计状态上的实际返回指令affine约束，尚未包含最终world限幅或物理安全含义。证据表明“上周期有效”不保证在当前变化后的约束仍有效，不能掩盖为模型已修正所以所有安全条件自动保持。

13个失败cycle：321–326、328、330–335，即6.42–6.70s。原生 [qpOASES Utils.c:900](/home/jjm/桌面/DSP/marsim_ws/src/no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Utils.c:900) 将simple status −2映射为不可行。恢复日志的“non-finite prediction/objective”标题是泛化故障分支，部分对应finite flag为false，并不证明这些周期出现NaN。当前不可行来源尚未做逐类约束受控归因，本次不调整权重/slack去使求解成功。

## 6. 求解成功率与实时性

|指标|历史修改前|本次修改前重跑 before|修改后 after|
|---|---:|---:|---:|
|求解失败次数|0|2|13|
|回退周期数|0|2|13|
|非零执行 slack 周期数|0|0|0|
|feedback 耗时均值 (ms)|2.100110|1.982151|1.954024|
|feedback 耗时 P95 (ms)|4.756001|4.398220|4.383507|
|feedback 耗时最大值 (ms)|14.858124|16.181095|9.216817|
|完整 NumSimMpc run 耗时均值 (ms)|6.734056|6.519610|6.542855|
|完整 NumSimMpc run 耗时 P95 (ms)|13.814067|13.113665|13.142092|
|完整 NumSimMpc run 耗时最大值 (ms)|26.646336|24.640051|20.595622|
|调度器周期处理均值 (ms)|8.623970|8.260680|8.266000|
|调度器周期处理最大值 (ms)|64.614700|29.542500|28.869400|
|调度器 >20 ms 的精确超时次数|28|11|21|


after求解成功率 1487/1500=99.1333%；before为1498/1500=99.8667%；历史为100%。失败率不应被round成“全部成功”。

三个耗时口径分开：

1. feedback_start/end 单调时钟包含实际condense/solve/expand及同模式的QP记录。
2. mpc_start/end 是完整 NumSimMpc run，含状态/参考/Field准备与相关诊断，范围较大。
3. 调度器summary统计周期处理（包括ROS callbacks、诊断/metrics写入、动力学积分等，排除为实时配速进行的sleep）。

NumSimMpc summary中的total_pipeline/core主要是feedback+preparation，未覆盖整个node周期。即使该列>20ms次数为0，也不能说没有deadline miss。准确调度器计数来自源码中的每周期累加与最终summary；不使用被1s节流的warning条数替代。after有21次、before有11次真实周期处理超时，各为1.4%与0.7333%。同时开启provenance/Field/full-QP capture，此处是**诊断配置下**的实时性，不能推断关闭记录后仍有同样超时，也没有无记录实验去证明默认实时性。

所有结果保留，未重跑挑选成功样本或静默改周期。控制器正常结束1500周期，roslaunch关闭其他感知进程时已有mutex teardown日志与先前一致，发生在数据完成后，不能当作上述QP失败原因。

## 7. 五个验收问题与停止建议

|问题|回答与证据等级|
|---|---|
|1. 测量角速度严格符合平面模型吗？|**已证实通过。** 真值/测量ωxy、UGV roll/pitch和全部MPC stage ωxy为0；yaw噪声保留，非目标控制噪声样本相同。|
|2. 补偿引起的指令突变明显减少吗？|**直接高度机制已证实消除；运行统计有改善证据。** 补偿P95约减少51.5%，world>.5次数84→14。但对整体因果改善只能限于非严格配对对照。|
|3. 相对指令与世界指令之间还会放大吗？|**仍有yaw/线速前馈差异，RP高度放大为0。** 小MPC变化仍可映射成更大world变化；最大两次大事件却已经在MPC返回中出现，不是映射创造出小→2m/s的RP异常。|
|4. tracking、物理间隙、成功率、实时性都正常吗？|**不能整体通过。** tracking RMS相对before略差；13次不可行/回退且当前affine条件负，21次deadline miss；动态采样几何间隙正，静态连续真值缺失。|
|5. 剩余大跳变来源是什么？|**已定位主要直接环节为MPC失败后的恢复输出。** yaw补偿仍有波动；具体不可行由哪些约束/RTI输入造成尚不能判断，未继续研究或修复。|

噪声校正本身是模型/工程修正，不能包装为控制算法创新。可以结束RP角速度噪声问题的排查；剩余控制波动仍值得独立诊断，尤其是QP可行性、当前约束下fallback及成功恢复命令连续性。这里只列已有证据支持的未解决问题，不开始滤波、slew limit、鲁棒化或QP重设计。

## 8. 数据、图表、验证与复现

新增唯一分析工具：[analyze_ugv_yaw_only_validation.py](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/scripts/analyze_ugv_yaw_only_validation.py)，含单UAV/参考/周期/精确真值关联/QP index/status防误用校验。三组数据共4500周期全部通过这些校验。独立核验复算command Δ统计与box SDF解析inside/face/corner案例，确认failed partial primal/dual排除；Python语法与git diff --check通过。模型速度传播验证低于7.82×10⁻⁹ m/s。after节点已在噪声修改时完成相关C++构建，本次before在隔离目录重编译并链接成功，编译日志保留。

- [after原始数据](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/after)：provenance、metrics、31500 Field stage行、1500 QP、sensors.bag、参数/源码/版本。
- [before独立重跑](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/before)，[独立构建记录](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/baseline_build/build_manifest.json)。
- [完整数值摘要](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/summary.json)、[三组对照CSV](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/run_comparison.csv)。
- [after逐周期真值与控制对照](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/after_per_cycle.csv)、[失败前后窗口](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/after_solver_failure_windows.csv)、[最大事件向量分解](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/after_top5_command_events.csv)、[Field所有stage](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/after_field_horizon.csv)。
- [指令/补偿/角速度/高度项/轨迹图](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/yaw_only_validation.png)、[实际响应和动态间隙图](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/analysis/actual_response_and_clearance.png)。
- [只读核验摘要](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/verification.json)、[交付清单和哈希](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/delivery_manifest.json)、[复现脚本](/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/reproduce.sh)。

从现有原始数据重新计算（新输出目录）：

    source /opt/ros/noetic/setup.bash
    source marsim_ws/devel/setup.bash
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
      marsim_ws/src/coni_mpc/scripts/analyze_ugv_yaw_only_validation.py \
      --run historical=/tmp/dsp_field_provenance_complete_20261012 \
      --run before=/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/before \
      --run after=/tmp/dsp_ugv_yaw_only_validation_20261012_dop2wi/after \
      --output-dir /tmp/dsp_yaw_only_analysis_new_unique_dir

完整复现脚本分别提供离线分析与新的软件仿真，使用归档二进制/源码与新的独立目录；每次DSP墙钟种子及异步时序仍可能不同，不能保证相同snapshot/失败次数/轨迹。历史目录未改写，当前控制/YAML未新增修改，无硬件动作、Git commit或push。
