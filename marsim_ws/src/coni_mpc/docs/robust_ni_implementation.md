# 因果非惯性不确定性与递归制动监督器

本次按用户提供的 `robust_mf_mpsc_codex_spec.md` 实现，不新增UGV预测器，
不改变ACADO维度或动力学。任务基线为1016分支 `cdb5c55205878d5f7cedd6799c61f14dc31265b4`。

## 模块与执行路径

1. `causal_ni_uncertainty.{h,cpp}` 接收原有21阶段的(a_g^N,omega,beta)，
   加速度/角速度/角加速度的连续时间上界，准确当前世界状态。
   按规范(6)–(21)处理完整二次角速度项、nominal一致性残差、ZOH精确旋转积分、
   5个20ms子区间和Metzler比较系统的3×3矩阵指数。
   每阶段输出整个100ms区间的上界，而非仅取末端。
   初始nominal角速度不等于已知当前角速度时拒绝认证；不修改原预测信号掩盖差异。
   固定已执行世界指令分支令命令耦合C=Omega=0。
2. `field_derivative_cert.{h,cpp}` 实现规范(22)–(33)：原C2场的精确h/g/H，
   不确定性球上解析G/H/T界、全快照L_F和rho_g。
   支撑检查覆盖R+E_P+epsilon，不使用nominal-only裁剪来建立证书。
   数学/数值有效性与覆盖/持久性metadata分开。
3. `recursive_safety_supervisor.{h,cpp}` 实现(37)–(46)，包含必要的(45b)。
   显式冻结静态证书的内容及ID，不能由新DSP快照自动替换。
   先检查mu=1，否则向q=0执行30轮射线二分并复验。
   缺证书、初始状态不属于K_F、数值错误或快照不匹配时，返回零世界速度指令，
   同时 `certified_safe=false`。返回制动不等于制动已被证明安全。
4. `robust_ni_runtime` 从原NI profile构造传播输入，在活动Field行加入 **+rho_g**。
   对返回计划用新控制序列重建tube，并记录精确nominal psi2-rho。
   使用原ACADO nominal frame重建世界状态，不能把相对状态和另一套UGV nominal中心混用；
   两套中心不一致时记录偏差，不声称计划已认证。
5. 数值节点在当前原始UGV位姿/速度/角速度下转换候选，原有限幅之后调用监督器，
   其q直接成为 `setControlAction` 的最终平移输入。缓存回退也必须经过此路径。
   没有监督后再次限幅、预测坐标重算或旧指令替换。

ACADO行使用warm-start候选构造rho，求解后控制序列可能改变；后验检查会重建tube，
但不把这些离散阶段的后验检查宣称为整个ACADO NLP的鲁棒递归可行性证明。
最终监督器的静态安全证明不依赖该仿射近似。
当前DSP阶段本身没有持久性/覆盖证书，其 `plan_exact_margin_valid` 不会被标真。
未来若要将ACADO计划本身认证，还须验证nominal中心在整个区间的一致性及仿射替代误差，
不能仅将此诊断flag改为true。

## 参数和认证条件

`parameters/num_sim_non_one_point.yaml` 新增 `robust_ni`：

- `enabled: false`：默认保留原跟踪行为。开启后每个20ms周期都会通过世界监督器。
- `delta_a/delta_omega/delta_beta`：标量或21项数组，单位分别为m/s²、rad/s、rad/s²。
  默认空数组表示未知，不是零误差，也不能用本次离散仿真样本最大值冒充连续上界。
- `safety_reserve_eta: 0.0`；`ray_bisection_iterations: 30`；`debug_csv`为可选输出路径。
- `require_certified_static_field: true`。关闭它不赋予动态DSP地图静态认证资格。
- `static_field/voxels_xyzo`：外部提供、可持久使用的完整世界系[x,y,z,occupancy,...]列表；
  `coverage_asserted`、`persistent_asserted`须有独立依据，默认false。
  这些是明确的建模假设输入，不是程序替用户验证了真实物理覆盖。

当前使用tau_xy=tau_z=0.2s、lambda=5；不同时间常数或执行周期不匹配使认证失效。
监督器输入是仿真器准确的当前P/W；实际硬件调用者必须自行满足准确测量和执行模型假设，
不能仅调用 `NumSimMpc::run()` 并绕过最终监督入口。
当前原配置含测量噪声，原预测omega_0也可能不等于准确当前omega；tube会显式拒绝这种不一致。
本次没有擅自把这些噪声设零或给缺失的误差界填经验常数。

安全适用范围为固定、持久且覆盖真实静态障碍的Field以及有效初始K_F。
P>=0.15仅是现有工程置信核心；它不证明覆盖。动态障碍严格保证仍需连续扫掠覆盖、
时间导数和持久安全终端域，本实现不宣称10Hz离散DSP切片已经满足这些前提。

## 验证记录

数学单元测试验证：零误差传播、精确NI残差、稳定小角积分、整个区间包络、
世界坐标动力学抵消、场导数与原函数一致、解析界的采样反例检查、静态K_F中的非零速度制动、
射线约束、下一步制动界以及缺失条件拒绝认证。采样仅检查实现，不是导数界的证明方法。

只运行一轮现有 `dsp_mpc_sim.launch` 闭环场景，开启新入口并记录CSV。
未提供误差界和静态覆盖证书，预期结果是未认证制动，不能展示为定理C的正向验证。
完整构建、测试、命令、参数、bag和源状态见 `audit_reports/robust_ni_20261023/`。
主诊断CSV是 `closed_loop_diagnostics.csv`；原始实时记录是 `robust_runtime.csv`。
合并表加入原circle轨迹的解析实际NI项及几何诊断，不拟合新预测器。
动态箱体间距按真实AABB计算；静态项是PCD采样点距离减UAV半径，不能叫连续实体表面净空。
真实误差界仍需要独立给定，仿真没有将它们“自动标定”。
