# 世界系物理包络与相对指令（2026-10-21）

本轮基于 `cbe03261ec9ce64a88015011f4aa236c9ae47e8d`，修改相对指令约束的物理含义。保留用户现有 YAML 数值、未跟踪证明文档、Field-HOCBF 平面定义和已有失败回退策略。

## 参数与模型

| 参数 | 新含义 | 当前本地数值 |
|---|---|---:|
| `max_v_xy` | 世界系速度指令的水平模长上界 | 5 m/s |
| `max_v_z` | 世界系速度指令竖直分量的绝对值上界 | 3 m/s |
| `max_yaw_rate` | 世界系 yaw 角速度指令绝对值上界 | 3.14 rad/s |
| `a_max_xy` | 世界系名义物理加速度水平模长上界 | 10 m/s² |
| `a_max_z` | 世界系名义物理加速度竖直分量绝对值上界 | 4 m/s² |

相对速度指令 `u_xyz` 和相对 yaw 指令的分量物理界已从生成模型、wrapper 和 controller 输出端删除。输入槽保留，使用 qpOASES `QPOASES_INFTY=1e20` 的无界哨兵；slack 的非负与上界保持。没有引入另一组相对指令物理上限。

```text
q_world = vg_world + R_WN * (u_non + omega_non × p_non)
a_world = R_WN * Lambda * (u_non - v_non)
yaw_rate_world = yaw_rate_non + omega_non.z
```

`a_world` 中不重复加入非惯性项：当前 `dot(v)=Lambda(u-v)+a_ni`，还原物理加速度时 `a_ni` 与坐标运输项抵消。这里约束的是名义模型加速度，不是实测推力/执行器加速度。yaw 相加公式对应当前重力对齐、平台只绕 z 轴旋转的 yaw-only 模型，不是任意 roll/pitch 下欧拉角速度变换。

## 生成契约与检查

重新生成了 ACADO；N=20、NX=7、NU=7、NOD=99、NPAC=12 不变，lbValues/ubValues 均仍为 140 项，condensed 约束 240 条。

| 每阶段路径行 | 当前表达式/边界 |
|---|---|
| 0 | `a_world.x²+a_world.y² <= a_max_xy²` |
| 1 | `-a_max_z <= a_world.z <= a_max_z` |
| 2 | `-max_yaw_rate <= yaw_rate_non+omega_non.z <= max_yaw_rate` |
| 3–9 | 原有 peer/Field-HOCBF 路径及顺序保留 |
| 10–11 | 原有世界系水平模长/竖直速度限制保留 |

OD 87–95 为 row-major R_WN，96–98 为平台世界速度，其余布局未变。水平速度/加速度优化界均内缩 0.001%，检查时仍按原物理界与 1e-7 容差（对应 m/s、m/s²、rad/s）检查控制阶段 0–19；如有残差，最多追加 16 次 preparation/feedback，仍不合格则拒绝候选解。终端节点没有控制输入，未额外声称约束终端实际速度。

QP 捕获格式升到 version 2，因为数组尺寸未变但前三条路径的含义改变。当前原生 replay/counterfactual 工具拒绝 v1 捕获，避免将旧模型交给新 solver；Python 离线审计继续支持 v1 与 v2，路径类别按版本解释。

## 验证结果与边界

- `coni_mpc` 包构建通过；11 项 C++ 测试、6 项 Python 测试通过。覆盖独立三维映射、倾斜静态坐标系可行冷启动、相对速度超过旧分量限制、平台 yaw 4 rad/s 的反向抵消、world yaw 上下界、水平加速度圆盘、实例隔离及不合格解拒绝。
- 新 v2 捕获的 5 个 QP 精确回放，H/A/边界/原始解/对偶解误差均为 0。独立 QP primal residual 最大约 2.00e-12。旧同尺寸 v1 捕获被当前 replay 明确拒绝。
- 两种坐标模式及与本地参数相同的 UGV yaw=2 rad/s 无障碍短仿真，共 900 条成功控制，无回退。已独立核对成功输入在 controller 输出端没有相对分量裁剪，世界速度/yaw 映射与 MPC 数据一致。

| 仿真用例 | 控制记录 | 成功/拒绝 | 控制器 P95/最大墙钟 ms |
|---|---:|---:|---:|
| noninertial | 300 | 300/0 | 6.303/10.512 |
| inertial | 300 | 300/0 | 5.410/6.318 |
| world_yaw | 300 | 155/145 | 83.521/92.919 |
| yaw_nominal | 300 | 300/0 | 1.828/2.104 |

额外 UGV yaw=4 rad/s、v=2 m/s 的压力仿真出现 **145/300 次残差拒绝与回退，最大控制器墙钟约 92.9 ms**，全部来自 UAV2；这些失败记录的 QP 状态均为 0，目标、状态及输入有限。实际是有限次 SQP/RTI 后非线性物理包络未达容差，不能把通用 recovery 日志标题 “non-finite” 当作 NaN/Inf。stage-0 world yaw 残差仅约 1e-12，不能据现有记录认定 yaw 本身或具体某个 stage 的水平加速度是拒绝源。

另一个故意使用远离动态可行域名义轨迹的倾斜坐标系冷启动压力用例，也在 16 次细化后拒绝候选（最大物理残差约 15.39）；可行初值的倾斜用例成功。没有通过增加迭代次数或放宽物理阈值掩盖这些边界。后续若要诊断具体原因，需逐 refinement 保存各约束最大残差/stage 与精确 states/inputs/OD/bounds。

外部 world 速度/yaw 限幅继续保护发布值，压力仿真回退期间确实使用了该保护；失败回退不保证名义加速度/HOCBF 可行。这些记录不构成全工况 20 ms 截止时间、实际执行器能力、连续时间安全或递推可行性证明。

原始证据与重跑脚本：`audit_reports/world_envelopes_20261021_203132/`，含 command.json、CSV、源码/配置哈希、生成日志、测试日志、QP 捕获、`run_smoke.py` 和 `verify_smoke.py`。上述仿真关闭障碍物/CBF以定向检查物理包络，未运行实物硬件。
