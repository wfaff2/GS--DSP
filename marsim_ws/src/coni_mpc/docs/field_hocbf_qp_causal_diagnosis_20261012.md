# DSP–Field-HOCBF–非惯性 MPC 真实 QP 记录与反事实因果验证

> 后续诊断已完成：Field profile 反事实改变 A 与 lbA/ubA；kkt 为 ACADO 代数标量；新运行的大幅世界指令增量主要来自带测量噪声的角速度补偿。历史解值不变，运行身份与证据范围见 [正式 provenance 报告](field_hocbf_world_command_provenance_report_20261012.md)。


日期：2026-10-12

## 1. 验收边界与实验身份

本次使用同一个新归档完成 QP 记录、原始 QP 重放和 A–D 真实 QP 反事实：

/tmp/dsp_field_replay_qp_20261012

HEAD 为 1e66bf5017cd93d07c2fa5a3c67b5cc2ab76f27f；运行前已有未提交工作树状态，归档内保存了 git_status.txt、source.diff、experiment.json、runtime_params.yaml 和源文件快照。运行命令通过环境变量启用诊断：

    CONI_MPC_QP_CAPTURE=/tmp/dsp_field_replay_qp_20261012/acado_qp_capture.bin \
    python3 marsim_ws/src/coni_mpc/scripts/run_field_replay_experiment.py \
      --output /tmp/dsp_field_replay_qp_20261012 --port 11431 --duration 30

该运行 1500/1500 个 MPC 周期求解成功。Field replay 重建误差、profile_b 误差和真值对齐误差均为 0。QP 记录文件大小约 641 MiB，包含 1500 条记录，未覆盖上一轮归档。

此前归档 /tmp/dsp_field_replay_current_20261012 只用于前一轮约束诊断；本报告中的 QP 结果只使用上述同一 QP 归档及其同一归档内的 field replay。

## 2. 实际求解链路审计

当前生成代码的关键路径是：

1. MpcController 更新 ACADO OnlineData、reference 和非惯性 profile。
2. MpcWrapper 调用 acado_preparationStep()，生成 RTI 线性化并 condense。
3. acado_feedbackStep() 调用 acado_condenseFdb()，随后调用 acado_solve()。
4. acado_qpoases3_interface.c 中实际构造 qpOASES-e QProblem(140, 200)，调用 QProblem_init(H,g,A,lb,ub,lbA,ubA,...)，再读取 primal/dual。
5. acado_expand() 将 QP 增量写回 ACADO 的状态和输入序列。
6. num_sim_non_one_point_node.cpp 将非惯性相对速度变换到 world，并做 world 绝对速度限幅。

因此本实现中没有持久化的 qpOASES hot-start QProblem 对象。所谓 warm start 主要是 ACADO 下一周期的 nominal x/u 轨迹；QP 求解本身每次通过 QProblem_init 从记录的 H/g/A/bounds 重新初始化。这个事实由生成的 acado_qpoases3_interface.c 直接确认。

## 3. 只读 QP 记录机制

修改文件：

- acado_model/quadrotor_mpc_codegen/acado_qpoases3_interface.c
- CMakeLists.txt
- test/qp_capture_replay_tool.cpp
- test/qp_counterfactual_tool.cpp

记录机制默认关闭；只有设置 CONI_MPC_QP_CAPTURE 才写文件。它位于实际 acado_solve() 内部，在 QProblem_init 前后捕获，不改变 H、g、A、边界、qpOASES Options 或控制输出。

每条记录包含：

- QP 维度：NV=140、NC=200；
- H 140×140、g 140、A 200×140；
- lb/ub、lbA/ubA；
- qpOASES 返回 status、nWSR、objective、KKT；
- qpOASES primal 140 和 dual 340；
- 调用前的 ACADO x/u/x0/od/y/yN/W/WN 和原始边界数组；
- 版本、固定数组布局和记录序号。

记录开销对比：

| 运行 | feedback 均值 | preparation 均值 | total pipeline 均值 |
|---|---:|---:|---:|
| 无 QP capture | 2.5076 ms | 0.7287 ms | 3.2362 ms |
| 开启 QP capture | 2.7233 ms | 0.7326 ms | 3.4559 ms |

平均增加约 0.22 ms；诊断写入不改变默认控制模式。

## 4. 原始 QP 精确重放

工具：

    marsim_ws/devel/lib/coni_mpc/qp_capture_replay_tool \
      /tmp/dsp_field_replay_qp_20261012/acado_qp_capture.bin

结果：

    records=1500
    exact_status_nwsr_and_solution=1500
    max_H_error=0
    max_A_error=0
    max_bounds_g_error=0
    max_primal_error=0
    max_dual_error=0
    max_objective_error=0
    max_kkt_error=0

重放步骤是：载入每条记录的 ACADO RTI 上下文，重新执行 preparationStep，比较实际 condensed QP；再调用同一生成的 acado_feedbackStep/qpOASES-e QProblem_init，比较 status、nWSR、primal、dual、objective 和 KKT。1500 条全部逐项一致。这满足原始 QP 冷启动独立重解的一致性验收；它没有声称存在不存在的 qpOASES 持久热启动状态。

## 5. 真实 QP 反事实构造

本次使用同一条 QP 记录、同一 ACADO x/u/reference/non-inertial 上下文，替换所有 21 个 prediction stage 的 Field-HOCBF profile，再重新执行真实 ACADO preparationStep + qpOASES QP solve：

- A_original：原始 profile；
- B_fixed_snapshot：同一周期状态和 nominal，替换为固定上一 DSP snapshot 的 profile；
- C_fixed_slice：同一 snapshot，替换为固定上一 prediction slice 的 profile；
- D_fixed_ids_current_state：固定上一周期候选 source IDs，使用当前 snapshot 粒子状态/占用重新计算 profile。
- E_previous_nominal_reference：固定当前 snapshot、当前 field profile、当前 x0/非惯性/感知输入，只把 ACADO 的上一周期 nominal x/u 和 reference y/yN 作为优化上下文替换，再真实重解 QP。

这是固定 RTI 上下文下的真实 QP 反事实，不是只比较 h/A/b。Field profile 的 profile_b 由同一回放记录和同一 Field-HOCBF 代码生成；跨 snapshot 没有把 PointCloud2 source_id 宣称为稳定物理身份。

工具：

    marsim_ws/devel/lib/coni_mpc/qp_counterfactual_tool \
      /tmp/dsp_field_replay_qp_20261012/acado_qp_capture.bin \
      /tmp/dsp_field_replay_qp_20261012/counterfactual_all.csv \
      322 330 1314

## 6. 三个事件的真实 QP 结果

下表的 u 是 ACADO 非惯性系 stage-0 解；同一事件的 world 坐标旋转和平移项固定，因此相对速度差的范数等于 world 速度差的范数。valid 表示 qpOASES status=0 且最大约束违约不超过 1e-7。

### 6.44 s，step 322

原始仿真世界命令跳变为 0.19835 m/s。该周期上一记录为 snapshot=63、source_stage=2；当前为 snapshot=64、source_stage=1。完整预测时域中 active Field-HOCBF stage 为 0–5。

|方案|valid|status|nWSR|QP objective|最大违约|相对 A 的 Δv_xyz|
|---|---:|---:|---:|---:|---:|---:|
|A|1|0|7|-144.0069|1.6e-12|0|
|B|1|0|5|-439.5342|7.2e-13|0.8885|
|C|1|0|6|-10.2799|1.4e-12|0.3320|
|D|1|0|5|-646.2955|1.9e-12|0.7104|

QP 矩阵变化主要出现在 condensed A：相对 A 的 max ΔA 分别约为 B=4.9602、C=4.9952、D=4.9540；H 和 g 的 max 差为 0。也就是说，感知/Field-HOCBF 变化通过真实约束 Jacobian 改变了 QP，且在相同 RTI 上下文中足以改变最优速度命令。

E 在该事件保持 field profile 不变，QP 的 max ΔA=0、max Δg=61.73，最优速度变化仅 0.0060 m/s。

### 6.60 s，step 330

当前 QP 归档的世界命令跳变为 0.32610 m/s；source_stage 在前一周期为 1，当前为 2。完整预测时域中 active stage 为 0–3。

|方案|valid|status|nWSR|QP objective|最大违约|相对 A 的 Δv_xyz|
|---|---:|---:|---:|---:|---:|---:|
|A|1|0|5|8.6813|2.0e-12|0|
|B|1|0|6|-199.1578|8.8e-13|0.4993|
|C|1|0|5|-109.6638|1.3e-12|0.1011|
|D|1|0|7|-203.2346|1.7e-12|0.4017|

max ΔA 为 B=4.9737、C=0.5190、D=0.7968；H 和 g 仍保持不变。prediction-slice 变化在真实 QP 中确实可以造成约 0.10 m/s 的直接控制差异；snapshot 和候选粒子状态的变化影响更大。

E 的 max ΔA=0、max Δg=44.66，最优速度变化为 0.0144 m/s。

### 26.28 s，step 1314

当前 QP 归档的世界命令跳变为 0.64591 m/s。当前 snapshot=264、age=0.1662 s、source_stage=2；stage-0 Field residual=5.7291，完整预测时域没有 active Field-HOCBF 行，所有 residual 均为正。

|方案|valid|status|nWSR|QP objective|最大违约|相对 A 的 Δv_xyz|
|---|---:|---:|---:|---:|---:|---:|
|A|1|0|0|-3.2519|0|0|
|B|1|0|0|-3.2519|0|0|
|C|1|0|0|-3.2519|0|0|
|D|1|0|0|-3.2519|0|0|

虽然 B/C/D 使 condensed A 的最大系数分别变化约 2.0551、0.4729、0.5664，但优化解、目标、KKT、nWSR 和相对控制完全不变。这个真实 QP 反事实排除了“当前 Field-HOCBF profile 变化是 26.28 s 命令突变直接原因”的解释。

E 保持当前感知和 Field-HOCBF profile，只替换上一周期 nominal/reference 后，QP 仍成功，max Δg=11.58，最优速度变化仅 0.0047 m/s；这也不足以解释在线 0.64591 m/s 跳变。

## 7. 感知、约束和最终命令的因果判断

### 已由真实 QP 验证的关系

- 在 6.44 s，snapshot、slice 和候选粒子变化均能改变真实 condensed A，并分别造成 0.332–0.888 m/s 的最优相对速度变化。
- 在 6.60 s，slice-only 反事实造成 0.101 m/s，snapshot-only 造成 0.499 m/s，固定候选 IDs/current state 造成 0.402 m/s。
- 26.28 s 即使 B/C/D 改变了 QP 约束矩阵，QP 最优解完全不变；E 的 nominal/reference 影响仅 0.0047 m/s。该时刻的 0.64591 m/s 在线命令变化不能归因于这些 Field-HOCBF 或已记录的 nominal/reference 变化。
- H 和 g 在这些 profile-only 反事实中保持不变；变化集中在路径约束的 condensed A。目标函数变化来自不同可行域下的最优解，而非代价矩阵变化。
- 当前生成 OCP 没有 inter-cycle Δu/slew 约束。它允许较大命令变化，但真实 QP 结果证明“允许变化”不是 26.28 s 事件的唯一原因。

### 仍需区分的因素

- 26.28 s 的真实 QP 反事实固定了捕获的 RTI 上下文；它排除了 Field-HOCBF profile 直接作用，但没有排除在线原始运行中的 reference/nominal trajectory 更新、ACADO 外部状态更新或非惯性 world 转换的同步变化。
- E 已完成为固定当前感知输入和 Field-HOCBF profile、替换上一周期 ACADO nominal/reference 的真实 QP 反事实。由于当前生成代码没有持久 qpOASES hot-start 对象，E 不能解释一个不存在的 qpOASES dual warm state；它量化了可观测的 nominal/reference 作用。
- 6.44/6.60 的 A–D 结果是固定 RTI 上下文下的真实 QP 反事实；它们证明约束变化能够造成命令变化，但不把它们宣称为在线世界命令跳变的唯一来源。

## 8. 结论与下一步

在线 DSP snapshot、prediction-slice 和候选 Voxel 集对最终命令的影响是事件依赖的：

- 6.44 s：在当前 QP 上是强因果因素，三类 profile-only 反事实均改变最优命令，最大约 0.8885 m/s。
- 6.60 s：slice-only 影响约 0.1011 m/s，snapshot-only 约 0.4993 m/s，候选状态/权重约 0.4017 m/s。
- 26.28 s：B/C/D 的真实 QP 影响为 0，E 的 nominal/reference 影响仅 0.0047 m/s；在线 0.64591 m/s 跳变来自尚未记录的同步上下文或坐标/命令链路因素，当前证据不支持把它归因于安全行。

在尚未提前选择创新方向的前提下，下一步最小工作是补齐 world-command provenance：每周期保存 ACADO post u、非惯性到 world 的变换前后 command 以及 node-level command provenance，用来定位 26.28 s 尚未解释的坐标/同步链路。当前不建议预先加入滤波、插值、slew limit 或新的鲁棒约束。

## 9. 复现与产物

- QP 归档：[acado_qp_capture.bin](/tmp/dsp_field_replay_qp_20261012/acado_qp_capture.bin)
- QP 归档清单：[qp_capture_manifest.json](/tmp/dsp_field_replay_qp_20261012/qp_capture_manifest.json)
- 同归档 Field replay：[replay.csv](/tmp/dsp_field_replay_qp_20261012/replay.csv)
- 同归档 A–D profile：[counterfactual_all.csv](/tmp/dsp_field_replay_qp_20261012/counterfactual_all.csv)
- 同归档真实 QP A–D：[counterfactual_qp_all.csv](/tmp/dsp_field_replay_qp_20261012/counterfactual_qp_all.csv)
- 事件 QP 对照图：[qp_event_counterfactuals.png](/tmp/dsp_field_replay_qp_20261012/qp_event_counterfactuals.png)
- 原始 QP 重放工具：marsim_ws/devel/lib/coni_mpc/qp_capture_replay_tool
- 真实 QP 反事实工具：marsim_ws/devel/lib/coni_mpc/qp_counterfactual_tool

## 2026-10-12 provenance 边界更正

后续边界审计发现，本报告早先“Field profile 变化集中在 condensed A、只改变 A”的表述不完整。对同一 RTI 上下文重新执行 ACADO preparation 后，Field-HOCBF profile 变化同时改变 Field-HOCBF 对应 condensed 行的 `A`、`lbA` 和 `ubA`；`H`、`g`、变量边界及其他路径约束行在 profile-only A–D 反事实中保持不变。原始 QP 数据和旧反事实结果未被修改。完整更正及逐行数值见 `field_hocbf_world_command_provenance_report_20261012.md`。

DSP、Field-HOCBF、非惯性动力学、代价、约束参数和默认控制模式均未改变；生成 ACADO 接口只增加了默认关闭的只读诊断捕获，另新增离线重放/反事实工具、报告和任务记录。

## 2026-10-12 正式 provenance 补充

正式新运行归档为 /tmp/dsp_field_provenance_complete_20261012。在 26.28 s，世界指令变化为 0.645709482 m/s；相对 MPC 输出仅变化 0.089984429 m/s，主要直接增量来自带测量噪声的 UGV 角速度叉乘位置补偿，限幅与最终应用未增加变化。求解/映射世界状态与角速度 profile0 完全一致，没有异步覆盖证据。这是新运行的机制定位，不能补造历史恰好 0.64591 m/s 的逐项日志。独立 KKT 残差与 1500+15 条 H 检查通过，历史反事实解值判断仍有效。condensed 行 199 为 stage19 的 Field path 行，非终端 stage20；局部路径顺序为加速度 0/1/2、legacy CBF 3/5/7、safe-set 4/6/8、Field 9。详细来源、数值、适用范围及研究建议见同目录正式 provenance 报告。
