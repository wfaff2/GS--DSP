# DSP-Map → Field-HOCBF → 非惯性 MPC 命令不连续因果诊断（2026-10-12）

## 实验身份与边界

本次诊断使用当前工作树 /home/jjm/桌面/DSP，分支 dsp/voxel-smooth-field-20261007，HEAD 1e66bf5017cd93d07c2fa5a3c67b5cc2ab76f27f。运行前工作树已有未提交改动；运行器保存了 git_status.txt、source.diff、参数和源文件快照。用户给出的 /tmp/dsp_field_replay_20261005132442 和其他可识别 replay 目录均不存在，因此不能把历史事件当作当前回放的同一实验。

新归档：/tmp/dsp_field_replay_current_20261012。启动命令：

    source marsim_ws/devel/setup.bash
    python3 marsim_ws/src/coni_mpc/scripts/run_field_replay_experiment.py \
      --output /tmp/dsp_field_replay_current_20261012 --port 11421 --duration 30

该运行是 50 Hz、30 s、non-inertial、frozen UGV rollout、Field-HOCBF 开启、DSP/LiDAR 在线输入；1500/1500 个 MPC 周期成功，碰撞监视为 0。诊断二进制的原始快照/周期重建误差均为 0，快照写入全部成功。构建命令 source /opt/ros/noetic/setup.bash && catkin_make --pkg coni_mpc -j2 通过。

## 活动实现审计

- DSP 快照：num_sim_mpc.cpp:581-681 校验时间戳和 map frame，读取 x/y/z、vx/vy/vz、occupancy、stage_index；每条消息生成一个最新快照，snapshot_id 单调递增。source_id 是该 PointCloud2 内的点序号（num_sim_mpc.cpp:667-674），不是跨快照物理目标 ID。
- slice 选择：num_sim_mpc.cpp:2012-2029 使用 field_dt=0.1 s 和 source_shift=round(field_age/field_dt)；stage k 取 source_stage=min(source_horizon,k+source_shift)，超出末 slice 才按发布速度外推。
- 非惯性变换与剪枝：num_sim_mpc.cpp:2071-2101 使用 car position/velocity、ω、β、a_car，再按半径、垂向差、占用、seed/separation/max_count 剪枝。
- Field-HOCBF：field_hocbf.h:129-255 是 occupancy-normalized LSE smooth-min；输出 h、hdot、Lf2、A=λ⊙∇h、b=-(Lf2+(α1+α2)hdot+α1α2h)。
- ACADO affine row：num_sim_mpc.cpp:2180-2201 计算 b_profile=b+A·(kv⊙v_nom+kp⊙p_nom)，写入 6×(N+1) profile；mpc_wrapper.cpp:337-359 写入 OnlineData row 80–84。生成模型约束见 quadrotor_model_thrustrates.cpp:464-476。当前 YAML 为 α1=α2=5、τxy=τz=0.2、slack_max=0。
- QP/命令：mpc_controller.cpp:252-344 更新 ACADO OnlineData 后做 warm/cold solve；mpc_controller.cpp:531-558 仅做输入绝对限幅；num_sim_non_one_point_node.cpp:3197-3221 做 non-inertial 到 world 的速度变换和 world 绝对限幅。
- inter-cycle limit：当前生成 ACADO OCP 没有把 previous_input_ 写成 Δu 约束；mpc_wrapper.cpp:530-534 只是保存上次解为下一周期锚点，setPreviousInput 也没有对应 ocp.subjectTo。因此当前实现不存在可证明的 inter-cycle command slew limit。

field_replay_tool 文档明确说明：它能无损重算约束，但不能重放 ACADO 的 QP warm start。当前 binary 还缺少完整 reference horizon、ACADO 工作区/active-set、上次 primal/dual、QP 矩阵和实际相对速度命令，所以不能构造同一 QP 初始化的真实反事实求解。

## 原始运行的基线量化

- 298 个有效 DSP snapshot；field_age 均值 0.1295 s、中位数 0.1232 s、最大 0.2043 s。
- stage-0 source slice：stage 1 共 1070 周期，stage 2 共 410 周期；source_shift 在约 0.15 s age 附近离散切换。
- stage-0 选中数均值 13.29、范围 0–30；全运行 Field-HOCBF active 行 24554/31500，slack 恒为 0。
- Field residual 非负，最小 0，最大 49.45；QP 解满足当前记录的 affine 不等式，但 residual 大时该行不是 active。
- 世界速度命令差 Δv 的中位数 0.249 m/s、P95 0.517 m/s、最大 1.479 m/s；Δv>0.5 m/s 有 91 次，>1 m/s 有 3 次。
- 相邻 stage-0 选中 source_id 的 Jaccard 中位数为 1，但 snapshot/slice 切换处可为 0。source_id 只在单个快照内有意义，不能解释为稳定物理身份。

## 两个优先时间窗口

当前回放的时间基准与用户给出的时刻一致到 0.02 s：6.60 s 是 step 330，26.28 s 是 step 1314。当前运行的最大跳变另发生在 6.44 s（step 322），所以把 6.44 s 作为实际大跳变对照。

### 约 6.44 s：实际最大跳变

step 321→322 的世界命令从 (-1.1736, 0.6822, 0.0089) 变为 (0.1246, 1.3884, -0.0402) m/s，Δv=1.47869 m/s。step 322 的 snapshot=64、age=0.0811 s、source_stage=1、选中 10 个；前一周期 snapshot=63 且 source_stage=2。相邻 source_id Jaccard=0。

原始 A 的 stage-0 约束：
h=-0.118892, hdot=0.143921, Lf2=-2.256221, A=(-1.364304,4.445091,0), b=3.789301, b_profile=59.242086。

|方案|h|hdot|Lf2|Axy|b|b_profile|selected|
|---|---:|---:|---:|---|---:|---:|---:|
|A 原始|-0.1189|0.1439|-2.2562|(-1.3643,4.4451)|3.7893|59.2421|10|
|B 固定上一 snapshot、当前 slice|-0.0938|0.1788|-2.8665|(-1.2136,4.5185)|3.4249|59.0254|11|
|C 当前 snapshot、固定上一 slice|-0.1113|0.1023|-2.4684|(-1.4513,4.4646)|4.2275|60.2832|10|
|D 固定上一周期 IDs、当前状态/权重|0.0489|1.2451|-2.5230|(4.1073,2.5545)|-11.1503|-0.9767|10|

6.38–6.68 s 窗口内相对 A 的最大变化：B 的 Δb=6.1433、ΔLf2=1.3777、ΔAxy=(1.1404,0.5443)；C 的 Δb=2.2319、ΔLf2=2.0674；D 的 Δb=18.7525、ΔLf2=5.8157、ΔAxy=(5.4716,7.6962)。这证明 perception 输入能显著改变安全行，但没有证明最终命令跳变全部由某一个输入单独造成，因为没有同一 QP 的反事实解。

### 约 6.60 s：prediction-slice 过渡窗口

step 330 仍是 snapshot=65、age=0.1415 s、source_stage=1，上一周期到该周期的 IDs Jaccard=1；source_stage 在 step 331（6.62 s，age=0.1614 s）跳到 2。6.60 s 的命令 Δv=0.40471 m/s，Field residual=0，说明该时刻 stage-0 行处于 active 边界。

step 330 的 A：
h=-0.096094, hdot=0.387729, Lf2=-2.905062, A=(-0.103123,4.645107,0), b=1.430114, b_profile=56.720573。

|方案|h|hdot|Lf2|Axy|b|b_profile|selected|
|---|---:|---:|---:|---|---:|---:|---:|
|A 原始|-0.0961|0.3877|-2.9051|(-0.1031,4.6451)|1.4301|56.7206|13|
|B 固定上一 snapshot、当前 slice|-0.0626|0.3647|-1.8201|(-0.6648,4.6212)|-0.2614|57.0981|13|
|C 当前 snapshot、固定上一 slice|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|13|
|D 固定上一周期 IDs、当前状态/权重|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|13|

6.50–6.68 s 窗口内 B 的最大 Δb=3.6223，C 的最大 Δb=1.3417，D 的最大 Δb=18.7525；source_stage 在 6.52、6.62、6.66 s 附近多次切换。slice 离散选择确实会改变约束，但 6.60 s 这一周期的约束变化主要来自当前 nominal/快照演化，不能把命令变化归因于尚未发生的 stage 1→2 切换。

### 约 26.28 s：命令变化但安全行不 active

step 1314 的命令由前一周期的 (-0.4496,-0.5137,0.0333) 变为 (-0.7835,-1.0640,0.0122) m/s，Δv=0.64401 m/s。snapshot=263、age=0.1216 s、source_stage=1、选中 20 个；step 1312 才发生 snapshot 262→263，step 1316（26.32 s）才发生 source_stage 1→2。step 1314 的 Field residual=5.9781、slack=0，当前 stage-0 安全行有明显正裕度，不是 active constraint。

A：
h=-0.025357, hdot=0.255496, Lf2=5.070243, A=(1.971000,-4.360364,0), b=-6.991263, b_profile=-32.153978。

|方案|h|hdot|Lf2|Axy|b|b_profile|selected|
|---|---:|---:|---:|---|---:|---:|---:|
|A 原始|-0.0254|0.2555|5.0702|(1.9710,-4.3604)|-6.9913|-32.1540|20|
|B 固定上一 snapshot、当前 slice|-0.0727|0.5102|5.4807|(2.7339,-4.0351)|-8.7647|-32.8026|14|
|C 当前 snapshot、固定上一 slice|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|与 A 相同|20|
|D 固定上一周期 IDs、当前状态/权重|-0.0280|0.2285|5.2457|(1.8928,-4.4033)|-6.8299|-32.1595|20|

在本次 run 中，26.28 s 的命令变化不能由 prediction-slice 变化解释（C=A），且该安全行不 active。B/D 会改变安全行，但实际 QP 命令反事实没有被记录，不能声称它们造成了 0.644 m/s 的命令变化。

## A–E 反事实可判定性

- A 已完成：原始完整链路的快照、每周期 stage、A/b/profile_b、h/hdot/Lf2、选中 IDs、snapshot age 和 source stage 均可无损重建。
- B 已完成为“固定 snapshot、改变 slice”的约束反事实，结果在 counterfactual_stage0_with_profile.csv；未重解 QP。
- C 已完成为“固定 slice、改变 snapshot”的约束反事实；未重解 QP。
- D 已完成为“固定上一周期 source IDs、使用当前 snapshot 中同 ID 的位置/速度/占用”的约束反事实；缺失 ID 显式计数，未跨快照伪造身份；未重解 QP。
- E（固定 perception inputs、只观察 optimizer response）不能从现有归档合法完成。缺少完整 reference window、ACADO 初始 primal/dual/workspace、QP Hessian/gradient/active-set 和相对系 command；把原始 command 贴到新 profile 不是 optimizer 反事实。需要控制器内新增只读记录器后重新运行，不能用当前数据声称 E 的因果结果。

## 因果结论与实现缺陷

1. 已验证会改变安全约束的因素：DSP snapshot 内容/年龄、离散 prediction slice、voxel 剪枝集合、粒子位置/速度/占用、非惯性变换和 nominal state 都会改变 h/hdot/Lf2/A/b_profile。D 在 6.44 s 的变化最大，说明集合内粒子状态/权重和 source identity 的变化足以重排 LSE 权重。
2. 已验证的实现级离散点：round(field_age/field_dt) 在约 0.15 s 处切换 source_stage；snapshot 更新又以约 0.1 s 重新开始 age。6.44 s 同时出现 snapshot 切换和上一周期 stage 2→当前 stage 1；6.62 s 出现明确 stage 1→2。该机制足以产生 profile discontinuity。
3. 已验证不存在的保护：当前 OCP 没有 inter-cycle Δu/slew 约束，只有绝对输入限幅。因此“命令跳变被 inter-cycle limit 触发/抑制”与当前代码不符。
4. 26.28 s 的因果结论受限：实际行 residual=5.978（不 active），C=A；本 run 不能把该命令变化归因于 slice transition 或 active Field-HOCBF 行。optimizer 的 warm-start/reference/non-inertial command conversion 是待验证假设。
5. source_id 是单快照点序号，跨 snapshot 不稳定；不能把 Jaccard=0 直接解释成物理障碍物消失，也不能把它当作稳定目标因果分析。

## 最小、可辩护的后续改进

优先增加只读 QP replay 记录，而不是调参：每周期保存完整 reference horizon、ACADO x/u/od/lb/ub/lbA/ubA、warm-start primal/dual（或 qpOASES 工作区）、相对系 command、world 转换前后 command、solver status/iterations/active rows。这样才能完成 E，并把“约束变化”与“optimizer active-set/command 变化”分开。

获得 E 证据前，最小算法候选是对 prediction-slice/snapshot handover 做时间连续化：保持原始 DSP 数据和 Field-HOCBF 公式不变，只避免 round(field_age/field_dt) 的硬切换，例如按实际 age 在相邻 slice 间连续插值并显式标记快照年龄。该建议针对已测得的离散 source_shift/profile 跳变，但当前证据不足以宣称它一定消除 26.28 s 命令跳变。不要先调 alpha、margin、slack penalty 或强行增加 slew limit，因为本次数据尚未证明这些是 26.28 s 的唯一原因。

## 归档与复现产物

- 原始归档：/tmp/dsp_field_replay_current_20261012
- 约束反事实：counterfactual_stage0_with_profile.csv
- 原始事件工具输出：event_self.csv、events_322_330_1314.csv
- 诊断程序：/tmp/field_cf.cpp，编译为 /tmp/field_cf
- 现有分析摘要：analysis_summary.json、truth_states.csv、replay.csv
