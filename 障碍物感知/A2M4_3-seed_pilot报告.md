# A2M4 sensing-constrained simulation 3-seed pilot 报告

## 1. 实验范围

- 时间：2026-08-09
- 工作点：$K_P=4$，在当前代码中对应 $V=W=2.0$
- 控制器：`noninertial_frozen` 和 `noninertial_stage`
- 随机种子：$1,2,3$
- 感知条件：`map`、`online_nominal`、`online_degraded`
- 运行总数：$3\times2\times3=18$
- 每次运行时长：$24.93\,\mathrm{s}$
- 无人机：UAV 1–3，每架 UAV 配置独立的虚拟激光雷达感知与 track 管理流程。

完整结果根目录：

`results/reviewer6_comment3_3_kp4_sensor_rplidar_a2m4/`

## 2. 参数加载核对

在运行日志中确认实际加载了以下参数：

| 参数 | Nominal | Degraded |
|---|---:|---:|
| 表面有效量程 | $0.15\text{--}6.0\,\mathrm{m}$ | $0.15\text{--}6.0\,\mathrm{m}$ |
| 水平视场 | $360^\circ$ | $360^\circ$ |
| 角分辨率 | $0.9^\circ$ | $0.9^\circ$ |
| 每次扫描射线数 | $400$ | $400$ |
| 扫描频率 | $10\,\mathrm{Hz}$ | $10\,\mathrm{Hz}$ |
| 可见性阈值 | $N_j^{\mathrm{hit}}\ge 3$ | $N_j^{\mathrm{hit}}\ge 3$ |
| 位置测量噪声标准差 | $0.01\,\mathrm{m}$ | $0.05\,\mathrm{m}$ |
| 端到端延迟 | $0.10\,\mathrm{s}$ | $0.30\,\mathrm{s}$ |
| 丢测概率 | $0$ | $0.10$ |
| track 保持时间 | $0.30\,\mathrm{s}$ | $0.50\,\mathrm{s}$ |
| Kalman 过程方差 | $10^{-6}$ | $10^{-6}$ |

## 3. 运行完整性

- $18/18$ 运行的 manifest 状态为 `ok`。
- $18/18$ 运行均产生 $7479$ 条 step 记录，即 $2493$ 个控制周期 $\times$ 3 架 UAV。
- $18/18$ 运行的碰撞计数均为 $0$。
- 在文本日志中未发现 `ERROR`、`FATAL`、`exception` 或 segmentation fault。
- 两种控制器在每个条件下使用相同的 experiment seed 和 noise seed：$1\leftrightarrow1$、$2\leftrightarrow2$、$3\leftrightarrow3$。

## 4. 核心结果

表中跟踪 RMS 报告为 3 个 seed 的均值 $\pm$ seed 间样本标准差；最小间距是全部 3 个 seed 中的最小真实 UAV–障碍物表面间距，不是均值。

| 感知条件 | 控制器 | 跟踪 RMS $(\mathrm{m})$ | 最小真实表面间距 $(\mathrm{m})$ | 实测丢测率 | 碰撞 |
|---|---|---:|---:|---:|---:|
| Map | Frozen | $0.362207\pm0.004606$ | $0.246798$ | 不适用 | $0/3$ |
| Map | Stage | $0.332620\pm0.024619$ | $0.248378$ | 不适用 | $0/3$ |
| Nominal | Frozen | $0.345837\pm0.009763$ | $0.244884$ | $0$ | $0/3$ |
| Nominal | Stage | $0.368305\pm0.025604$ | $0.255534$ | $0$ | $0/3$ |
| Degraded | Frozen | $0.374038\pm0.017642$ | $0.237434$ | $0.100817$ | $0/3$ |
| Degraded | Stage | $0.371741\pm0.025292$ | $0.246017$ | $0.100707$ | $0/3$ |

## 5. 感知链路检查

- 每个在线运行均执行 $750$ 次扫描，每次扫描 $400$ 条射线。
- Nominal 的实测丢测率为 $0$，与设置一致。
- Degraded 中 Frozen 和 Stage 的实测丢测率分别约为 $10.08\%$ 和 $10.07\%$，与目标 $10\%$ 一致。
- Nominal 中 active-track 非空控制步比例为 $99.599\%$。
- Degraded 中 active-track 非空控制步比例为 $98.797\%$。
- 每架 UAV 同时活跃的 track 平均约为 $5.10\text{--}5.27$，小于 6 是因为视线遮挡、量程及 track 保持逻辑，不是将全局地图直接传入控制器。
- 运行末端尚在延迟队列中的测量不会在仿真结束后继续交付，因此 generated 数略高于 delivered 数；这些不应计为额外 dropout。

## 6. safety-event 覆盖边界

当前低间距阈值为

\[
\tau=0.215\,\mathrm{m}.
\]

本 pilot 的全局最小真实表面间距为

\[
d_{\min}^{\mathrm{truth}}=0.237434\,\mathrm{m}>\tau.
\]

因此 18 个运行中均没有触发 low-clearance sample/episode。这意味着：

- 安全事件记录列已正常输出；
- 但本次 3-seed pilot 不能用来估计“低间距时无 track”的发生率。

## 7. 运行时警告

唯一反复出现的警告是 `MPC scheduler overrun`。按每个运行的 `rosrun.log` 去重统计，共有 $144/44874=0.321\%$ 个调度周期超过 $10\,\mathrm{ms}$，最大为 $72.1462\,\mathrm{ms}$。仿真时间由

\[
t_k=k\Delta t,\qquad \Delta t=0.01\,\mathrm{s}
\]

确定，不是由墙钟时间积分，所以这些警告没有造成步数丢失或仿真时长不完整。如果后续需要把 CPU 运行时作为论文结果，正式运行建议设置 `JOBS=1` 减少并行竞争。

## 8. 结论

该 3-seed pilot **通过功能性验收**：实验组合、参数加载、扫描触发、遮挡/可见性、高斯噪声、延迟队列、dropout、Kalman track 与输出统计均正常。可以进入 20-seed 正式实验。

但 3 个 seed 不足以作为控制器优劣的统计证据，不应根据本表宣称 Stage 或 Frozen 更好。
