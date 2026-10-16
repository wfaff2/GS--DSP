# Field-HOCBF 世界速度指令来源诊断与 QP 数值审计（2026-10-12）

## 结论与证据等级

**已证实：** 新归档的 cycle 1313→1314（26.26→26.28 s）世界速度指令变化为 **0.645709482 m/s**。ACADO expand 后及控制器返回的相对速度只变化 0.089984429 m/s；大幅增量首先出现在非惯性角速度叉乘位置的补偿环节。世界限幅和最终应用没有再改变该指令。该周期实际使用的测量角速度变化 0.220142652 rad/s，而原始模拟 UGV 角速度完全没有变化。代码明确在每次状态估计中向 UGV 角速度加入高斯噪声，再将同一个测量量用于 MPC 非惯性 profile 和世界速度映射。

**有证据支持：** 本次事件的主要直接来源是测量角速度噪声经位置杠杆臂放大进入速度前馈。按下文明确的有序向量分解，角速度变化贡献范数为 0.486920394 m/s，对最终变化方向的投影占 74.734%。这是一项在已记录 u、p、R 上的代数贡献，不是独立改变噪声后重新运行闭环得到的总因果效应。噪声也会影响 MPC 输入和未来状态，不能把贡献范数相加或把 74.734% 宣称为普适因果比例。

**尚不能判断：** 历史运行中恰好 0.64591 m/s 的那一条指令没有完整 provenance，无法补造其逐项来源。本次新运行出现相近事件，并验证了同一代码路径中的具体机制；同一仿真时刻不代表随机感知、状态轨迹或 snapshot 相同。对历史记录的 B/C/D 无影响与 E 仅约 0.0047 m/s 的结论保留在历史实验范围内。

**已证实：** CSV 的 kkt 是生成 ACADO 的未归一化代数标量，不是 qpOASES 原生驻点/可行性/互补残差。独立审计历史原始 1500 条 QP 和 15 条真实反事实 QP 的四类残差均小，H 均通过对称性和 Cholesky 检查。没有发现这些记录的数值最优性异常。

**已证实：** profile-only B/C/D 不仅改变 condensed A，也改变同一 Field-HOCBF 行的 lbA、ubA；H、g、lb、ub 及其他路径行不变。已在旧因果报告增加更正，历史实验文件未覆盖。

## 1. 范围、实验身份与归档

本任务只增加默认关闭的诊断记录、分析工具、验证和文档，没有修改 DSP 感知/预测算法、Field-HOCBF 公式、MF-MPSC 动力学、代价、约束、增益、slack、控制周期或求解设置。没有添加滤波、插值、hysteresis、slew limit；没有真实硬件调用、Git commit 或 push。用户 YAML 没有改动，原有未提交文件保留。

- 历史 QP 与反事实输入：/tmp/dsp_field_replay_qp_20261012，1500 条 QP。数值及边界审计使用这一历史数据，不能与新运行逐点混用。
- 本次正式 provenance：[/tmp/dsp_field_provenance_complete_20261012](/tmp/dsp_field_provenance_complete_20261012)，原配置、DSP + MARSIM LiDAR、30 s、50 Hz、1500 个完整周期，含同时采集的 QP、Field replay 和传感器 bag。
- 诊断开关验证：/tmp/dsp_field_provenance_gate_off_20261012 与 /tmp/dsp_field_provenance_gate_on_20261012，各 2 s、100 周期。为排除异步感知的不确定性，仅这两次验证禁用 DSP/LiDAR/碰撞感知；它们不用于主事件结论。
- 更早不完整 provenance 试跑 /tmp/dsp_field_provenance_20261012 不作为本报告主证据。
- HEAD：1e66bf5017cd93d07c2fa5a3c67b5cc2ab76f27f，分支 dsp/voxel-smooth-field-20261007，工作树包含未提交研究改动。正式运行节点二进制 SHA256：199a92c56d699e0e3887a54ae368e8043c3ec7981ef69d328033cfb3de251b89。
- 正式运行开始于 2026-10-12 16:45:48 +08:00；[experiment.json](/tmp/dsp_field_provenance_complete_20261012/experiment.json) 保存启动命令、版本、二进制/运行时源码哈希；source/、source.diff、git_status.txt、runtime_params.yaml 保存当时快照。分析工具在运行后有完善，最终版本另存 analysis_source/ 与 delivery_manifest.json，避免将后改工具误标为运行时版本。
- 角速度估计噪声标准差 0.0435 rad/s、线速度 0.025 m/s；noise_seed 的代码默认值为 1，按 quad_id 偏移，来源为源码和运行配置，不假定 ROS 参数表必然含默认值。障碍物 seed 与估计噪声 seed 分别处理。
- ROS 时间戳是 wall/ROS 时间，sim_time 是调度器的逻辑仿真时间；steady clock 只用于同进程顺序与耗时。三种时基单独记录，未跨时基直接相减。

控制节点正常写完 1500 周期后退出。关闭其余传感器进程时，roslaunch.log 出现既有 Boost mutex teardown 异常；它发生在周期数据完成之后，未截断日志，不能作为 26.28 s 跳变根因。

## 2. 实际控制生成流程

1. 单线程调度器先执行 ROS spinOnce，获取本控制周期 UAV 状态快照，更新 UGV odometry，再逐个 UAV 执行控制器。控制计算结束后才统一 advanceOneControlStep；该路径没有在 solve 与 mapping 之间再次 spinOnce 或启用异步 spinner。来源：[调度器](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:6573)、[周期快照](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:6616)。
2. NumSimMpc::run 构造相对估计、参考、Field profile 和非惯性 profile。其中 [genRelativeEstimate](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/coni_mpc/num_sim_mpc.cpp:989) 将逐周期高斯噪声加到原始 UGV 线速度和角速度。它们是明确的模拟测量量，不是日志采集凭空引入的误差。
3. MpcController 调用 wrapper；[updateLocked](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/acado_mpc/mpc_wrapper.cpp:515) 设置 x0，执行 feedbackStep。生成 solver 在 feedback 中完成 condenseFdb → solve → expand；[expand](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/acado_model/quadrotor_mpc_codegen/acado_solver.c:10801) 将 QP 增量加到 nominal u。记录中 QP primal 是 condensed 增量，不能直接当最终速度。
4. 控制器取 predicted_inputs.col(0)，在 [updateControlCommand](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/acado_mpc/mpc_controller.cpp:532) 对每轴非惯性速度、yaw rate 与 slack 做现有绝对限幅。失败路径 [makeFailureFallback](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/acado_mpc/mpc_controller.cpp:207) 返回上一有效指令或零值；节点随后仍沿原映射路径使用该返回值。惯性模式失败另有保持相对位置的映射分支，本次非惯性成功事件未走该分支。
5. 节点从本次控制器取得 UGV 姿态、位置及上述测量线/角速度，从模拟器取得当前 UAV 世界位置，实际计算：

   p_N = R_WNᵀ (p_UAV_W − p_g_W)

   c_N = ω_g_N × p_N

   v_pre_W = v_g_W + R_WN (u_return_N + c_N)

   代码见 [世界映射](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:3305)。这里 mapping 的 p_N 来自模拟器世界状态，MPC x0.position 来自带估计噪声的相对状态，二者不应强制相等；已经分别记录，不能把这种既有测量差异认定为异步状态更新。
6. 对世界 XY 速度做范数限幅，Z 做标量限幅；yaw 加 ω.z 后限幅。这和上一步控制器逐轴输入限幅是不同环节。
7. 将四维最终值写入 [setControlAction](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/src/num_sim/num_sim_non_one_point_node.cpp:3388)，诊断从 getControlAction 读回实际值。**本次实际运行节点的末端是模拟器控制输入，没有额外 ROS 速度命令 publisher。** CSV 中 published_*、publish_mono_sec 是沿用的字段名，表示最终应用值/应用时刻，不冒充存在 ROS 发布链路。预测轨迹/odometry 的 ROS 发布不是执行速度指令。

生成 qpOASES-e 接口每次新建 QProblem，使用 QProblem_init 冷启动；不存在应当重放却遗漏的跨周期 qpOASES 热启动对象。ACADO nominal、参考、初态与上周期控制 anchor 均是需保留的 RTI 上下文。

## 3. 新增诊断实现与默认关闭验证

启用参数 world_command_provenance_csv，默认空字符串；环境变量 CONI_MPC_WORLD_COMMAND_PROVENANCE 可作为空参数的后备。两层 launch 已转发参数。只有输出流成功打开才为每个 UAV 打开诊断；关闭时不执行诊断矩阵拷贝、额外转换或时钟采样。wrapper 与 NumSimMpc 的记录开关同样默认 false。求解输入、数学表达式和控制器设置保持原路径。

文件与职责：

- include/acado_mpc/mpc_wrapper.h、src/acado_mpc/mpc_wrapper.cpp：在实际 feedback 前后抓取 nominal、QP 增量、expand 后 stage-0、feedback 状态及单调时钟。每次 controller run 先 reset，未执行 feedback 时状态为 −999，避免把上一周期输出伪装为新解。
- include/acado_mpc/mpc_controller.h、src/acado_mpc/mpc_controller.cpp：只读返回 previous-valid fallback 是否实际使用，公开 wrapper 诊断；不改变回退决策。
- include/coni_mpc/num_sim_mpc.h、src/coni_mpc/num_sim_mpc.cpp：记录实际使用的 FieldSnapshot ID/stamp、估计位置和 omega_profile0，而非求解后再读取一个可能已变的 snapshot。
- src/num_sim/num_sim_non_one_point_node.cpp：96 列、17 位有效数字 CSV；统一键 (uav_idx, cycle_id)，sim_time、UAV/UGV odom stamp、pipeline/feedback 起止、应用 ROS/steady 时间、状态/转换矩阵/补偿/限幅前后/最终读回，以及原始与测量 UGV 速度。
- scripts/validate_world_command_capture.py：将实际 QP 与 provenance 逐周期交叉检查，并验证开/关诊断归档；旧 QP 格式没有 UAV ID，交叉检查明确只接受单 UAV，拒绝多 UAV 模糊关联。
- scripts/analyze_world_command_provenance.py：相邻周期分解，自动事件检测与指定窗口，不按相同仿真时刻拼接不同运行。
- scripts/qp_numeric_audit.py、scripts/validate_qp_diagnostics.py、test/qp_counterfactual_tool.cpp：独立残差、全元素边界审计和 full-QP 反事实输出。
- test/world_command_provenance_test.py、test/qp_numeric_audit_test.py：实际周期关联/恒等式与含已知最优解、错误 primal/dual、非有限数据的独立测试。
- scripts/run_field_replay_experiment.py：独立目录、配置/代码/运行时参数与传感器归档。

输入限幅前值为 acado_u*，限幅后实际返回为 mpc_cmd_*；QP 状态 qp_status 是 ACADO feedback 返回的 qpOASES simple status，不是完整 returnValue 枚举。实际成功运行所有状态为 0，没有回退。失败字段与 −999 路径经代码审核/构建，但本次没有人为制造闭环求解失败，不把未走过的失败路径称为运行验证完成。

**关闭验证：** 两次独立 100 周期软件运行中，开/关诊断的完整 QP capture 逐字节一致，四维实际指令完全一致，关闭时 provenance 文件不存在。见 [gate_validation.json](/tmp/dsp_field_provenance_complete_20261012/gate_validation.json)。这一受控验证支持默认数学输入和控制行为不变，不声称两个异步 DSP 运行会逐时间戳相同。

**关联验证：** 正式运行的 1500 周期中，记录的 nominal + QP 增量与 expand 后输入一致；QP x0 的估计位置、OD[68:71] 的 stage-0 omega 与 provenance 相同；映射世界 UAV 状态与 solve 输入 odom 世界状态相同。见 [cross_capture_validation.json](/tmp/dsp_field_provenance_complete_20261012/cross_capture_validation.json)。

## 4. 26.28 s 逐环节对照

|周期 / 仿真时刻|expand 后速度 = MPC 返回速度 N (m/s)|UGV 测量角速度 N (rad/s)|世界限幅前 = 限幅后 = 实际应用速度 (m/s)|snapshot|
|---|---|---|---|---|
|1313 / 26.26 s|(0.096656236, 0.365243202, 0.013400176)|(0.043788614, -0.123697085, 0.181595113)|(-0.445320035, -0.497897037, 0.035835188)|265|
|1314 / 26.28 s|(0.174468005, 0.409389718, 0.003724881)|(-0.009594326, 0.049445669, 0.056556577)|(-0.782008009, -1.048340420, 0.011462556)|265|
|1315 / 26.30 s|(0.113405237, 0.429336195, 0.009669243)|(-0.026637303, 0.007597664, 0.131223794)|(-0.537877027, -0.931338463, 0.057087296)|265|


三个周期均 qp_status=0、solve_ok=1、fallback_used=0。输入限幅、世界限幅与最终后处理均未介入事件。1313→1314 的实际环节差值：

|环节|相邻周期差值二范数|
|---|---:|
|QP expand 后 / MPC 返回相对速度|0.089984429 m/s|
|UGV 测量世界线速度|0.092517548 m/s|
|UGV 测量角速度|0.220142652 rad/s|
|ω × p 补偿项|0.486757024 m/s|
|u + ω × p|0.574394536 m/s|
|旋转后的相对世界速度|0.573456609 m/s|
|世界限幅前、限幅后、实际应用|0.645709482 m/s|

原始 UGV 角速度在两周期都是 (0, 0, 0.14285714285714285) rad/s。因此 Δω_measured 在这里等于 Δ估计噪声；测量值来自源码中的明确噪声注入。直接在补偿项出现大增量，R 的小变化进一步耦合，随后加测量 vg 得到最终变化；不能笼统把跳变归于旋转矩阵、QP 解或限幅。

为避免耦合项重复计数，a 为上一周期、b 为当前周期，规定下列顺序分解；c_clip = v_after − v_pre，c_post = v_applied − v_after：

Δv_applied = Δvg + Ra Δu + Ra(Δω × pa) + Ra(ωb × Δp)
             + (Rb − Ra)(ub + ωb × pb) + Δc_clip + Δc_post

该式严格望远镜相消，但分配交叉耦合的次序不唯一。这里 Δu 使用返回值，已经包含输入限幅。表内向量可以相加，范数不能相加：

|有序向量贡献（世界系）|x|y|z|二范数 (m/s)|
|---|---:|---:|---:|---:|
|Δvg|0.00725248035|-0.0922327538|0.000131760101|0.0925175481|
|Ra Δu|-0.0385014106|-0.0807541178|-0.00967529481|0.0899844291|
|Ra (Δω × pa)|-0.307042476|-0.377626375|-0.0146529579|0.486920394|
|Ra (ωb × Δp)|0.000271919642|-3.5026397e-06|-0.000176139987|0.000324002862|
|ΔR (ub + ωb × pb)|0.00133151167|0.000173365937|0|0.00134275056|
|Δ限幅修正|0|0|0|0|
|Δ应用后处理|0|0|0|0|


事件向量闭合误差 5.55×10⁻¹⁷ m/s；全部相邻周期最大闭合误差 5.83×10⁻¹⁶ m/s。角速度噪声贡献的方向投影定义为 c_noiseᵀ Δv / ||Δv||²，本事件 0.747340217；这是给定其余已记录量的条件分解，未实施“移除噪声后闭环重跑”的算法变更。

自动检测阈值为 ||Δv_applied||₂ > 0.5 m/s，本次共 80 个事件；同时输出 6.44、6.60、26.28 s 以及自动大事件的前后窗口。新运行在 6.44 与 6.60 s 的指令变化分别为 0.37544、0.12858 m/s，它们不同于历史运行，历史反事实的幅值不能移植给新轨迹。

[命令来源变化图](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_provenance.png) 同时显示全程事件与 26.28 s 放大窗口；[重点事件对照](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_event_comparison.csv) 保留所有向量分量，而非只保留范数。

## 5. 时序与同步审计

正式运行 cycle 1314 的 UAV stamp 为 1791794779.1085618，UGV stamp 为 1791794779.1085734，差约 11.683 µs。全程平均/95 分位/最大差为 17.503/31.483/63.181 µs。这两个 wall/ROS stamp 由顺序调用分别赋予；它们不是相同数值，但所代表的状态都在同一个逻辑控制周期取得。不能仅依据微秒 stamp 差推断混用了不同物理采样时刻。

求解世界 UAV position 与映射 position 的逐项误差为 0，profile omega0 与 mapping omega 误差为 0；反馈结束 ≤ controller run 结束 ≤ 最终应用，cycle_id 连续、应用值无覆盖。代码与完整日志均未发现 solve/map 间异步更新、缓存覆盖或跨控制周期混配。估计相对位置和真实映射位置的差由既有估计噪声生成，不是日志把不同时间数据拼接造成。

DSP 是独立感知发布者，snapshot stamp 可以早于控制状态，且有 prediction slice/extrapolation；这是实际异步感知输入，已经记录。本事件 snapshot=265、stamp=1791794778.927489，1313/1314 未更换；Field replay 给出 stage0 age≈0.181798 s。它不是控制输入 odom 的时间戳，不应强制三者相同。

正式新运行 1313/1314/1315 的 20 个 Field condensed 行对偶值均为 0，最小下界余量分别 0.238452361、6.510439259、3.643555824。见 [new_run_event_field_rows.json](/tmp/dsp_field_provenance_complete_20261012/qp_audit/new_run_event_field_rows.json)。Field 日志中 active=1 表示 profile/行启用，**不是在解处约束绑定**。因此新事件也没有安全行绑定的证据。

**判断：** 已证实存在分别赋予的 stamp 与异步 DSP 感知时效；当前控制路径没有被证实存在导致此事件的状态同步缺陷。本结论限于当前软件调度器与新运行，不外推到其他节点、真实硬件或其他线程模型。

## 6. KKT 数值定义与独立审计

### 6.1 CSV 的 kkt 是什么

[生成 acado_getKKT](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/acado_model/quadrotor_mpc_codegen/acado_solver.c:12182) 计算：

kkt_acado = |gᵀ x| + Σ(yv_i > ε) |lb_i yv_i| + Σ(yv_i < −ε) |ub_i yv_i|
                    + Σ(yc_j > ε) |lbA_j yc_j| + Σ(yc_j < −ε) |ubA_j yc_j|，ε=10⁻¹²。

x 为 QP 增量，yv 为变量边界 signed dual，yc 为一般约束 signed dual。该标量不包含 Hx+g−yv−Aᵀyc，不是归一化残差，也不是调用 qpOASES_getKktViolation。已独立重算该表达式，与历史 CSV/capture 最大差 6.82×10⁻¹³；最大记录值 3104.138737 本身不说明不最优。测试含“真实残差为零但此标量很大”的已知解析 QP。

### 6.2 符号、排列和残差

本地链接的 qpOASES-e [Utils.c](/home/jjm/桌面/DSP/marsim_ws/src/no_catkin_compilation/acado-stable/external_packages/qpoases3/src/Utils.c:658) 明确使用 y[0:140] 为变量边界，y[140:340] 为 200 个一般约束；正 dual 对应 lower，负对应 upper。不是通过试符号寻找较小残差猜出来的。

实际 QP 是 min 0.5 xᵀHx + gᵀx，lb≤x≤ub，lbA≤Ax≤ubA。令 z=(x,Ax)，l=(lb,lbA)，u=(ub,ubA)，signed dual d=(yv,yc)：

- 原始可行性：max(0, max有限侧(l−z), max有限侧(z−u))。
- 驻点：||Hx + g − yv − Aᵀyc||∞。
- 对偶符号可行性：正 d 要有有限下界、负 d 要有有限上界；无对应侧的违规 dual 最大值。两侧都有有限边界时 signed dual 不要求统一正号；等式行可使用任一符号，互补残差检查其等式值。
- 互补：正 d 检查 |(z−l)d|，负 d 检查 |(z−u)d|，取最大值。本独立脚本忽略 |d|≤10⁻¹²；本地 native 函数阈值为 1000×机器 epsilon，不将二者误称为完全相同实现。

额外检查阈值影响：历史 1500 条只有一个非零 dual 小于等于 10⁻¹²（cycle333、变量18，−2.22×10⁻¹⁶），该变量值和所选上界均为0，因此不忽略时的互补乘积仍为0；15条反事实没有此类小 dual。当前数据的阈值未掩盖互补异常。

有限侧以 |bound|<10²⁰ 判定，与库 INFTY 相符。生成代码的 ±10¹² 路径大数边界仍被当作记录 QP 的有限侧，而非任意删除。输入/一般线性约束两类均覆盖，非有限数组直接拒绝。

相对指标定义：

- primal 每一侧按 max(1, |z_i|, |bound_i|) 缩放后取最大值，避免一个 10¹² 边界掩盖其他约束违约。
- stationarity 除以 max(1, ||Hx||∞, ||g||∞, ||yv||∞, ||Aᵀyc||∞)。
- dual sign 除以 max(1, ||d||∞)。
- complementarity 除以 max(1, ||x ⊙ yv||∞, ||Ax ⊙ yc||∞)。该值是诊断缩放，不替代原始残差。

### 6.3 审计数值与适用范围

|历史原始 1500 条的指标|最大原始值（∞范数/最大违约）|最大相对值|
|---|---:|---:|
|primal|2.56772×10⁻¹²|2.48396×10⁻¹²|
|stationarity|3.41061×10⁻¹³|2.33161×10⁻¹⁵|
|dual sign|0|0|
|complementarity|3.90406×10⁻¹⁰|1.54284×10⁻¹¹|

15 条历史真实反事实 QP 最大 primal=1.95660×10⁻¹²、stationarity=3.97904×10⁻¹³、dual sign=0、complementarity=6.64193×10⁻¹¹。所有 1515 条 H 对称并通过 Cholesky，状态为成功；历史目标函数重算误差最大 2.39×10⁻¹²。这些独立检查共同支持记录的凸 condensed QP 解在浮点精度下满足最优性，而不是仅凭重放一致推断。

接口使用 Options_setToMPC，允许 qpOASES 内部正则化；生成 solver 的 Hessian/可能的 LM 项已包含在记录 H 中。未发现额外未记录的用户级缩放；重新 preparation 的 H 与记录逐元素一致，使用记录 H 的驻点残差也小。内部 regVal 没有归档，**不能断言每次内部正则化系数为零**，也不改求解设置追求更好数字。本次没有发现应继续调查的残差/非正定异常。10¹² 边界在 double 中约 10⁻⁴ 的量化差需解释，不能误报为可行性故障。

1500 条精确重放直接证明 RTI 记录、矩阵/边界重建、同求解流程的状态与解一致；它单独不能证明最优性或安全性。加上独立残差与 H 检查，也仅证明所记录的有限维凸 QP 的数值最优性，不证明原始非线性问题、Field 模型保守性、采样间安全、碰撞安全或闭环稳定性。

## 7. A–E condensed 矩阵与边界审计

下表全部来自历史 capture 在当前真实反事实工具中的重解，而非新运行 26.28 s 的 QP。A 为原始；B 固定上一 snapshot，C 固定上一 slice，D 固定上一候选 ID 并用当前状态/权重计算，E 替换上一 nominal/reference 上下文。B/C/D 固定当前 RTI 其余量，仅替换 Field OD[80:85] 的 profile/启用位。E 还替换 nominal x/u、y/yN、W/WN；记录中权重一致，实际差来自上下文。

各 Δ 为相对同事件 A 的逐元素最大绝对差；Δv_xyz 只取前三个速度分量，不混入 rad/s 的 yaw。15 条完整 QP 的矩阵、边界、primal、dual 已保存 counterfactual.bin，索引 0–14 按 counterfactual.csv 标签排列。

|时刻 / 方案|ΔH|Δg|ΔA|Δlb|Δub|ΔlbA|ΔubA|Δv_xyz (m/s)|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|6.44 / A|0|0|0|0|0|0|0|0|
|6.44 / B|0|0|4.96017373|0|0|48.807133|48.8071289|0.888483375|
|6.44 / C|0|0|4.99523028|0|0|49.851917|49.8518066|0.331977804|
|6.44 / D|0|0|4.95396109|0|0|57.5240243|57.5239258|0.710421617|
|6.44 / E|0|61.7336395|0|0.460341497|0.460341497|3.584898|3.58483887|0.00600325888|
|6.60 / A|0|0|0|0|0|0|0|0|
|6.60 / B|0|0|4.97370466|0|0|50.0126166|50.0126953|0.499282895|
|6.60 / C|0|0|0.51897625|0|0|1.65587264|1.65600586|0.101101397|
|6.60 / D|0|0|0.79679695|0|0|2.54736509|2.54748535|0.401708693|
|6.60 / E|0|44.6553896|0|0.344625719|0.344625719|2.31223904|2.31237793|0.0144102206|
|26.28 / A|0|0|0|0|0|0|0|0|
|26.28 / B|0|0|2.05505248|0|0|7.14568057|7.14562988|0|
|26.28 / C|0|0|0.472944666|0|0|1.14946821|1.14941406|0|
|26.28 / D|0|0|0.566414296|0|0|1.24317182|1.24328613|0|
|26.28 / E|0|11.5798675|0|0.0590417889|0.0590417889|0.295208945|0.295208945|0.00466342643|


Generated path 行实际顺序经 evaluatePathConstraints 输出确认：局部 0/1/2 为加速度，3/5/7 为三个 legacy CBF，4/6/8 为相应 safe-set，9 为 Field-HOCBF（[out[9]](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/acado_model/quadrotor_mpc_codegen/acado_solver.c:4206)）。不是将 3–5 与 6–8 错当连续分组。condensed 行 j=10×stage+local，stage=0…19，共 200 行。行 199 是 **stage19 的 Field path 行**，不是终端 stage20。输入 profile 存储有 21 个 stage，不代表最后一个是 condensed path 行。

|时刻 / 方案|A、lbA、ubA 发生变化的 Field stage|对应 condensed 行|
|---|---|---|
|6.44 / B|0–18|9, 19, …, 189|
|6.44 / C|0–18|9, 19, …, 189|
|6.44 / D|0–18|9, 19, …, 189|
|6.60 / B|0–16|9, 19, …, 169|
|6.60 / C|0–15|9, 19, …, 159|
|6.60 / D|0–15|9, 19, …, 159|
|26.28 / B|0–19|9, 19, …, 199|
|26.28 / C|0–18|9, 19, …, 189|
|26.28 / D|0–19|9, 19, …, 199|


B/C/D 的变化只落在这些 Field 行，其他路径约束行逐元素一致。E 的 A/H 不变，但 g、变量边界 lb/ub 和路径边界变化；路径边界局部行只涉及 0/1/2 与 9。其中 E@6.44 的 Field stage0–17、E@6.60 stage0–15、E@26.28 stage0–19，三次加速度边界 stage0–19。所有变化元素（含列、前后值、差值、stage/local）共 4888 项，见 [matrix_element_differences.csv](/tmp/dsp_field_provenance_complete_20261012/qp_audit/validation/matrix_element_differences.csv)。阈值为 |Δ|>10⁻¹²；完整二进制仍保留阈值以下的数值。

为什么边界会变：preparation 重新 evaluatePathConstraints，profile 同时改变路径函数值与导数；生成代码在 condensed lbA/ubA 中减去函数值，feedback 再减去初态/线性化修正。可见 [边界构造](/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/acado_model/quadrotor_mpc_codegen/acado_solver.c:9458)。因此只替换 A 而冻结边界并不等于当前实现的真实 profile 反事实。工具已经逐变体 load RTI → preparation → feedback → expand，未手工拼装混合 QP。

A 的七类矩阵/边界及 primal/dual 与对应历史原始 capture 完全相同。26.28 s 的 B/C/D 虽改变 Field 行的矩阵和边界，但 primal/dual 与 A 逐位相同；E 的速度差 0.004663426 m/s。因此历史“Field 变化在 6.44/6.60 可改变最优控制、在 26.28 的这些 B/C/D 中无影响”结论仍成立。应该更正的是“只改变 A”的解释，不是旧解值。

这些是固定 RTI 上下文的一步 QP 受控反事实，不是让整条轨迹重新运行的闭环反事实。不能据此认定移除 Field、噪声或换参考后整段轨迹也保持一致。

## 8. 验证与性能开销

|验证|结果|
|---|---|
|相关 coni_mpc C++ / 节点 / QP 工具构建|通过，保留编译日志；未将既有 warnings 误称为零 warning|
|FieldHocbfTest|3/3 通过|
|FieldReplayDiagnosticsTest|2/2 通过|
|新增 QP 独立解析/损坏数据单元测试|4/4 通过|
|provenance 完整周期及损坏应用值验证|1500 周期通过，故意损坏发布/应用值被拒绝|
|历史 QP 精确重放|1500/1500，H/A/g/所有边界、status/nWSR、primal/dual/objective/kkt 差为 0|
|正式新运行 QP 精确重放|1500/1500，同项差为 0|
|新运行 Field replay|1500 周期，14 个 snapshot unavailable 周期被明确标记，31206 个 stage，verification_failures=0|
|历史原始与反事实数值/边界断言|1500+15 条通过；H 均 Cholesky 成功；4888 个变化元素|
|开/关诊断受控比较|100 周期完整 QP 逐字节一致，实际控制四维一致|
|新运行 provenance 与实际 QP 交叉验证|1500 周期 status/nominal/increment/expand/x0/profile0 一致|

测得节点诊断数据采集均值/95 分位/最大为 0.000761/0.001461/0.007382 ms；CSV 序列化为 0.100350/0.242980/0.397475 ms；二者逐周期合计为 **0.101111/0.244570/0.398890 ms**。均值约占 20 ms 控制周期的 0.506%。原始逐周期值见 world_command_provenance.csv.timing.csv。

此测量覆盖节点内采集和序列化，**不包括 wrapper/NumSimMpc 的少量拷贝/时钟、最终关闭文件 flush、timing CSV 自身的完整写入，也不包括另行启用的 full-QP/Field capture 开销**。因此不能声称它是所有层的完整观测开销上界。求解总耗时受感知、OS 与 QP 活跃集影响，没有用不同随机运行的总耗时差冒充纯记录成本。默认关闭消除了这些诊断采集/写入，仍有少量 bool 分支。本次未改变周期去容纳日志。

## 9. 四个问题的明确回答与下一步

**问题 1：** 在本次可复现软件运行中，约 0.64571 m/s 的变化主要首先形成于 ω_measured × p 的非惯性运动补偿；MPC 返回只变 0.08998 m/s，世界限幅/实际应用没有引入额外变化。原始 UGV 角速度不变、测量角速度跳变和源码噪声注入共同定位其直接输入来源。历史恰好 0.64591 的逐项数据缺失，不能声称恢复了该旧周期。

**问题 2：** 状态消息的 stamp 不完全相同，DSP 感知有时效差；但当前调度器求解/映射世界状态与 omega_profile0 一致，日志没有异步覆盖或跨周期混配证据。微秒 stamp 差不是本事件同步缺陷的证明，不能据此实施同步补丁。

**问题 3：** 旧 kkt 被当成“真实 QP KKT 残差”会造成错误解释，数字本身是 ACADO 已定义的标量；本次独立残差验证未发现数值最优性故障。边界解释需要改为 A、lbA、ubA 同时变化；真实 condensing 已做，历史 B/C/D/E 的受控解值与局部因果判断仍成立。安全性和闭环因果不能由精确重放代替证明。

**问题 4：** 最有证据支持的下一研究问题是：带测量误差的非惯性速度前馈如何经过位置杠杆臂影响最终世界指令，以及相对系 MPC 内约束与最终世界系执行限制如何保持一致。首个最小研究步骤应为固定记录/随机种子，定量分析 R(δω×p)+δvg 的误差传播和敏感性，并通过真正的一变量受控反事实区分直接前馈与通过 MPC/闭环传播的间接作用；再研究是否需要世界指令层的约束一致性。当前不实施滤波、增益调整或 slew limit，也不预先包装成新的安全理论。

若之后发现估计量 frame_id/单位/噪声注入与预期不符，应首先归类为工程问题。本次没有证据把明确配置的合成测量噪声称为软件 bug，也没有完成通用三维真实 UGV 传感器坐标约定的验证。

## 10. 产物与复现命令

- [原始 provenance CSV](/tmp/dsp_field_provenance_complete_20261012/world_command_provenance.csv) 与 [JSON](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_provenance.json)。
- [全部相邻周期分解](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_all_deltas.csv)、[重点事件窗口](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_event_windows.csv)、[变化图](/tmp/dsp_field_provenance_complete_20261012/analysis/world_command_provenance.png)。
- [历史 QP 数值审计](/tmp/dsp_field_provenance_complete_20261012/qp_audit/original/qp_numeric_audit.json)、[反事实数值审计](/tmp/dsp_field_provenance_complete_20261012/qp_audit/validation/counterfactual_numeric_audit.csv)、[七数组变化及逐行边界](/tmp/dsp_field_provenance_complete_20261012/qp_audit/counterfactual.csv)、[逐元素变化](/tmp/dsp_field_provenance_complete_20261012/qp_audit/validation/matrix_element_differences.csv)。
- [可复现完整命令](/tmp/dsp_field_provenance_complete_20261012/reproduce.sh)、[交付元数据与哈希](/tmp/dsp_field_provenance_complete_20261012/delivery_manifest.json)、[验证摘要](/tmp/dsp_field_provenance_complete_20261012/verification/verification_summary.json)。

构建/验证可在仓库根目录执行，归档脚本中每次使用新的目录，不能复用历史路径：

    source /opt/ros/noetic/setup.bash
    source marsim_ws/devel/setup.bash
    (cd marsim_ws && catkin_make --pkg coni_mpc -j2)
    python3 marsim_ws/src/coni_mpc/scripts/run_field_replay_experiment.py \
      --output /tmp/dsp_field_provenance_new_unique_run \
      --port 11471 --duration 30 --provenance --capture-qp

离线分析可以仅使用已有正式新运行及历史 QP；完整命令见 reproduce.sh。复现的是配置、求解流程、记录和分析方法，而非承诺异步感知重新运行产生完全相同的 snapshot 或时钟戳。所有实验仅在软件仿真或离线程序中执行。
