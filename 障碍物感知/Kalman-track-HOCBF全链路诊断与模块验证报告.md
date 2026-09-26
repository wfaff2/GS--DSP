# Kalman–track–HOCBF 全链路诊断与模块验证报告

## 1. 修改边界

本次修改只增加诊断状态、CSV 输出和验证脚本，没有修改以下控制行为：

- 传感器可见性、遮挡、噪声、延迟和 dropout 生成逻辑；
- Kalman filter 的状态方程以及参数 $Q$、$R$；
- track 删除条件和 $T_{\mathrm{hold}}$；
- HOCBF 障碍物排序、最多三个障碍物选择、约束或安全半径。

仿真真值只进入诊断结构和日志，不进入 tracker、MF-MPSC 或 HOCBF。

## 2. 已修正的诊断问题

### 2.1 测量年龄

原日志使用

$$
N_{\mathrm{current}}-N_{\mathrm{delivery}}
$$

作为 track age。刚交付的延迟测量会因此显示为 $0$，即使它是在 $d$ 帧前生成的。

现在分别记录：

$$
N_{\mathrm{age}}=N_{\mathrm{current}}-N_{\mathrm{generation}},
$$

$$
N_{\mathrm{since\ delivery}}=N_{\mathrm{current}}-N_{\mathrm{delivery}},
$$

$$
N_{\mathrm{delay}}=N_{\mathrm{delivery}}-N_{\mathrm{generation}}.
$$

三者满足：

$$
N_{\mathrm{age}}-N_{\mathrm{since\ delivery}}=N_{\mathrm{delay}}.
$$

track 的删除逻辑仍按“距离上次交付的时间”执行，因此本次修正不改变 $T_{\mathrm{hold}}$ 行为。

### 2.2 Kalman 和重建轨状态

每个 active track 现在记录：

- 最后一次原始位置测量 $\tilde p_o^W$；
- Kalman 位置 $\hat p_o^W$；
- 协方差 $P$；
- innovation、innovation covariance 和 Kalman gain；
- measurement update 次数；
- track instance 编号；
- 本周期是否创建、更新或重新建轨。

重新建轨后的 `track_instance` 大于 $1$。初始建轨和重新建轨没有先验预测，因此 `innovation_valid=0`；后续 Kalman correction 才有有效 innovation。

### 2.3 世界坐标和求解器坐标

旧列 `active_obstacle_x/y/z/radius` 为兼容已有脚本而保留，其实际含义仍是 solver 坐标。新增明确字段：

- `active_obstacle_world_*`：进入变换前的世界坐标障碍物；
- `solver_active_obstacle_*`：进入 HOCBF 的 solver 坐标障碍物；
- `solver_active_static_truth_*`：相同静态障碍物的仿真真值；
- `solver_active_static_track_*`：相同静态障碍物的 Kalman track；
- `truth_uav_world_*` 和 `estimated_uav_world_*`：UAV 真值与控制器估计位置。

### 2.4 距离误差分解

对 HOCBF 当前绑定的同一个静态障碍物，记录：

$$
d_{\mathrm{truth}}
=\left\|p_{u,\mathrm{truth}}^W-p_{o,\mathrm{truth}}^W\right\|
-r_u-r_o,
$$

$$
d_{\mathrm{track}}
=\left\|p_{u,\mathrm{truth}}^W-\hat p_o^W\right\|
-r_u-r_o,
$$

以及原有的 $d_{\mathrm{solver}}$。新增误差为：

$$
e_{\mathrm{track}}=d_{\mathrm{track}}-d_{\mathrm{truth}},
$$

$$
e_{\mathrm{solver|track}}=d_{\mathrm{solver}}-d_{\mathrm{track}},
$$

$$
e_{\mathrm{solver|truth}}=d_{\mathrm{solver}}-d_{\mathrm{truth}}.
$$

其中 $e_{\mathrm{track}}$ 主要反映障碍物测量和 Kalman 位置误差；$e_{\mathrm{solver|track}}$ 包含 UAV 状态估计与坐标变换的影响；$e_{\mathrm{solver|truth}}$ 是最终进入 HOCBF 的总距离误差。若 solver 当前绑定的是另一架 UAV，静态障碍物专用字段保持无效值，不强行与静态障碍物真值配对。

## 3. 输出文件

每次仿真现在自动产生：

- `metrics_steps.csv`：每 UAV、每控制周期的 HOCBF 绑定障碍物和全链路误差；
- `metrics_sensing_tracks.csv`：每 UAV、每控制周期、每个 active 静态 track 的详细状态。

可复用验证脚本为：

```bash
python3 scripts/validate_kalman_track_hocbf_diagnostics.py \
  --step-csv <metrics_steps.csv> \
  --track-csv <metrics_sensing_tracks.csv>
```

## 4. 模块验证结果

### 4.1 C++ 单元与回归测试

执行：

```bash
cmake --build build --target run_tests -j2
```

结果：$16/16$ 通过。新增覆盖：

- 生成、交付和更新周期的分离；
- innovation、innovation covariance 和 Kalman gain；
- 删除后重新获取同一障碍物时的 `track_instance=2` 和重建轨标志；
- 原有 FOV、量程、遮挡、三条射线、随机流、dropout、噪声和静态 Kalman 收敛回归。

关键数值：

- $10000$ 次 Degraded dropout 测试：观测率 $0.101300$；
- 二维高斯噪声标准差：$\sigma_x=0.049912\,\mathrm{m}$，$\sigma_y=0.049891\,\mathrm{m}$；
- 静态 Kalman 稳态位置 RMSE：$0.009638\,\mathrm{m}$；
- 最终协方差 trace：$0.000324\,\mathrm{m}^2$。

### 4.2 三种感知模式短时冒烟测试

Map、Nominal 和 Degraded 三种模式均成功完成短时仿真，无运行失败。

- Map：$2400$ 条逐-track 记录，`tracker_online=0`，track 中心误差最大值为 $0$，测量/Kalman 专用字段保持无效值；
- Nominal：$780$ 条逐-track 记录，$78$ 条交付更新，$2$ 条创建，$76$ 条有效 correction；
- Degraded：$1143$ 条逐-track 记录，$93$ 条交付更新，$5$ 条创建，$88$ 条有效 correction。

Degraded 更新行验证得到：

$$
t_{\mathrm{measurement\ age}}=0.3\,\mathrm{s},\qquad
t_{\mathrm{since\ delivery}}=0,\qquad
t_{\mathrm{delay}}=0.3\,\mathrm{s}.
$$

CSV 与公式自动校验结果：

- `metrics_steps.csv`：$134$ 列，所有行列宽一致；
- `metrics_sensing_tracks.csv`：$54$ 列，所有行列宽一致；
- track 中心误差公式最大残差：$1.235\times10^{-6}\,\mathrm{m}$；
- solver–truth 距离误差公式最大残差：$1.77\times10^{-16}\,\mathrm{m}$；
- solver–track 距离误差公式最大残差：$1.0\times10^{-6}\,\mathrm{m}$；
- 旧 `active_obstacle_x/y` 与新增明确 solver 坐标的最大差值：$0$。

CSV 默认保留六位小数，因此 $10^{-6}\,\mathrm{m}$ 量级残差来自文本舍入，不是计算链不一致。

## 5. 当前结论

全链路诊断已可区分：

$$
\text{原始测量误差}
\rightarrow
\text{Kalman track 误差}
\rightarrow
\text{UAV 状态/坐标误差}
\rightarrow
\text{HOCBF 最终距离误差}.
$$

当前验证只证明诊断模块和数据恒等式正确，不构成控制器鲁棒性证据，也没有据此修改 Kalman 参数。下一步应使用本版本重新运行正式 $3$-seed pilot，再根据重新建轨时刻、低净空时刻和 solver 绑定障碍物的误差决定是否调整 $Q$、$T_{\mathrm{hold}}$、多帧初始化或协方差安全缓冲。
