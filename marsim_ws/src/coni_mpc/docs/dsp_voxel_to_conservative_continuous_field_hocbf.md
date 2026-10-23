# DSP 体素到物理保守连续安全场 $h_F$ 的全流程规范与实现指南

> 原始设计材料，非当前运行算法或闭环安全证明。2026-10-23 已按用户要求引入低占据率
> 平滑降权、C2 紧支集和按时间回退；当前保证只针对高置信体素几何包络，见
> [当前实现说明](field_confidence_and_fallback_20261023.md)。本文的全正概率支持集下界
> 和“绝对不可能碰撞”等表述不能用于描述当前控制器。

> **文档定位**：本文档旨在彻底解决动基座无人机（UAV）跟踪无人车（UGV）时，从前端动态空间划分（DSP）离散体素地图到后端控制屏障函数（CBF）之间的**几何覆盖断层、距离虚假高估、Z轴人为截断以及求解失败回退不安全**四大核心痛点。
> 
> 本文档给出了满足**严格物理保守性（零高估）**、**分段二阶连续可微（Piecewise $C^2$）**的真 3D 安全场 $h_F$ 的完整数学推导，并提供工程落地的 C++ 替换代码。

---

## 1. 架构全景流程图

```mermaid
flowchart TD
    A["DSP 动态占据地图点云 (Dynamic Voxels)<br/>质心 c_m, 速度 v_m, 加速度 a_m, 占据率 P_m"] --> B["Step 1: 确定性几何外接球覆盖<br/>体素半径 r_m = (√3/2) ℓ_m, 机体外壳 r_uav"]
    B --> C["Step 2: 动基座时空运动学预测对齐<br/>前瞻时间 τ = k·dt，将外推状态变换到 UGV 动基座坐标系"]
    C --> D["Step 3: 确定性物理感知域截断<br/>证明 R_cert >= (v_max² / 2a_max) + r_uav + r_m 外体素不可达"]
    D --> E["Step 4: 真 3D 欧氏正则化保守净间隙<br/>s_m = √(||p - c_m||² + ε²), d_m^cert = s_m - ε - r_uav - r_m"]
    E --> F["Step 5: 严格保守移位 LogSumExp 连续安全场 h_F<br/>h_F = d_min - σ·ln(∑ κ_m·exp(-(d_m^cert - d_min)/σ)) - d_margin"]
    F --> G["Step 6: 高阶 Lie 导数解析微分与 ACADO 仿射行组装<br/>A = Λ^T ∇_p h_F,  b = -(L_f² h_F + (γ1 + γ2)·hdot + γ1·γ2·h_F)"]
```

---

## 2. 核心数学链条推导

### 2.1 确定性几何外接球覆盖（从离散点到连续实体）

真实物理障碍物为三维连续占据体 $\mathcal{O}_{\mathrm{real}} \subset \mathbb{R}^3$。DSP 地图输出分辨率为 $\ell_m$ 的正方体体素质心 $\mathbf{c}_m$。

* **体素外接球半径**：
  $$r_m = \frac{\sqrt{3}}{2} \ell_m \tag{1}$$
  若 DSP 体素分辨率为 $\ell_m = 0.15\,\text{m}$，则体素外接球半径为 $r_m \approx 0.13\,\text{m}$。
* **无人机机体外接球**：$r_{\mathrm{uav}} = 0.15\,\text{m}$。
* **确定性几何包络覆盖条件（假设 A2）**：
  $$\boxed{\mathcal{O}_{\mathrm{real}}(t) \subseteq \bigcup_{m\in\mathcal{I}(t)} B(\mathbf{c}_m(t), r_m)} \tag{2}$$
  对任意障碍物内部真实点 $\mathbf{q} \in \mathcal{O}_{\mathrm{real}}$，必存在某个体素球 $m$ 满足 $\|\mathbf{q} - \mathbf{c}_m\|_2 \le r_m$。

---

### 2.2 真 3D 欧氏正则化保守净间隙（消除距离虚高）

原代码强制执行 `delta_p.z() = 0.0`，实为二维柱面投影，无法应对 3D 空间避障。重构方案彻底恢复全 3D 向量计算。

1. **三维相对位移**：
   $$\Delta \mathbf{p}_m = \mathbf{p} - \mathbf{c}_m \in \mathbb{R}^3$$
2. **平滑模长正则化**（引入 $\epsilon = 1.0\times 10^{-4} > 0$ 消除原点导数奇异）：
   $$s_m = \sqrt{\|\Delta \mathbf{p}_m\|_2^2 + \epsilon^2} \tag{3}$$
   光滑外法向（模长恒小于 1）：
   $$\mathbf{n}_m = \nabla_{\mathbf{p}} s_m = \frac{\Delta \mathbf{p}_m}{s_m} \in \mathbb{R}^3 \tag{4}$$
3. **严格保守物理净间隙**：
   $$\boxed{d_m^{\mathrm{cert}} = s_m - \epsilon - r_{\mathrm{uav}} - r_m} \tag{5}$$

> **【物理保守性证明】**：  
> 由于 $\sqrt{x^2 + \epsilon^2} - \epsilon \le x$ 对所有 $x \ge 0$ 严格恒成立，故有：
> $$s_m - \epsilon \le \|\Delta \mathbf{p}_m\|_2 \implies d_m^{\mathrm{cert}} \le \|\mathbf{p} - \mathbf{c}_m\|_2 - r_{\mathrm{uav}} - r_m$$
> 再由式 (2) 三角不等式，对任意 $\mathbf{q} \in \mathcal{O}_{\mathrm{real}}$，存在体素 $m$ 使：
> $$d_m^{\mathrm{cert}} \le \|\mathbf{p} - \mathbf{q}\|_2 - r_{\mathrm{uav}}$$
> 对真实障碍物点取下确界，严格有：
> $$\boxed{d_{\min}^{\mathrm{cert}} = \min_{m\in\mathcal{I}} d_m^{\mathrm{cert}} \le D_{\mathrm{phys}}(\mathbf{p})} \tag{6}$$
> **结论：减去 $\epsilon$ 彻底抵消了正则化引起的虚假测距偏大，该距离度量处处保守！**

---

### 2.3 严格保守移位 LogSumExp 连续安全场 $h_F(\mathbf{p})$

为了保持物理距离下界，**绝不能除以 $\sum P_m$（Log-Mean-Exp 算子会导致测距被大幅高估），也绝不能在每个体素前乘以 Wendland 截断核（衰减核会导致障碍物被虚化抹杀）**。

1. **证书加权系数（保底权重）**：
   $$\kappa_m = 1 + \alpha P_m \ge 1, \quad (\alpha \ge 0) \tag{7}$$
2. **移位 LogSumExp 安全场定义**：
   令 $d_{\min}^{\mathrm{cert}} = \min_{m\in\mathcal{I}} d_m^{\mathrm{cert}}$，
   $$\boxed{h_F(\mathbf{p}) = d_{\min}^{\mathrm{cert}} - \sigma \ln \left( \sum_{m\in\mathcal{I}} \kappa_m \exp\left(-\frac{d_m^{\mathrm{cert}}(\mathbf{p}) - d_{\min}^{\mathrm{cert}}}{\sigma}\right) \right) - d_{\mathrm{margin}}} \tag{8}$$
   其中 $\sigma > 0$（例如 $\sigma = 0.12$），$d_{\mathrm{margin}} > 0$（例如 $0.2\,\text{m}$）。

> **【零高估定理（Zero-Overestimation Theorem）】**：  
> 取达到最小值的体素索引 $j$（即 $d_j^{\mathrm{cert}} = d_{\min}^{\mathrm{cert}}$）：
> $$\sum_{m\in\mathcal{I}} \kappa_m \exp\left(-\frac{d_m^{\mathrm{cert}} - d_{\min}^{\mathrm{cert}}}{\sigma}\right) \ge \kappa_j \exp(0) = \kappa_j \ge 1$$
> 取自然对数后，其值恒大于等于 0：
> $$\ln \left( \sum_{m\in\mathcal{I}} \dots \right) \ge 0 \implies -\sigma \ln \left( \sum_{m\in\mathcal{I}} \dots \right) \le 0$$
> 代入式 (8)，严格得到：
> $$\boxed{h_F(\mathbf{p}) \le d_{\min}^{\mathrm{cert}} - d_{\mathrm{margin}} \le D_{\mathrm{phys}}(\mathbf{p}) - d_{\mathrm{margin}}} \tag{9}$$
> **结论**：
> $$\boxed{h_F(\mathbf{p}) \ge 0 \implies D_{\mathrm{phys}}(\mathbf{p}) \ge d_{\mathrm{margin}} > 0}$$
> **只要控制器保证 $h_F \ge 0$，在物理真实世界中四旋翼外壳距离任何障碍物实体的净间距必定大于等于 $d_{\mathrm{margin}}$，绝对不可能发生穿透碰撞！**

---

### 2.4 理论辨析：为什么不能在 LogSumExp 中直接乘平滑截断核（如 Wendland 核）？

若为了消除边缘跳变而在体素项前乘紧支集衰减核 $\phi(q_m) \in [0, 1]$：
$$S_{\mathrm{flawed}} = \sum_{m} \phi(q_m) \kappa_m \exp\left(-\frac{d_m - d_{\min}}{\sigma}\right)$$
* **破坏保守性**：由于 $\phi(q_m) \le 1$，求和项 $S_{\mathrm{flawed}}$ 变小，取负对数后 $-\sigma \ln(S_{\mathrm{flawed}})$ 反而**被人为抬高**；
* **物理严重后果**：当无人机以 $5\,\text{m/s}$ 高速冲向处于截断边缘（$q \approx 0.9$）的实体墙时，$\phi(q) \approx 0.005$ 把这堵墙的危险度稀释了 200 倍！CBF 误判为极其安全（$h_F \gg 0$），拒绝输出刹车指令，导致无人机因制动距离不足直接撞墙。

---

### 2.5 高阶 Lie 导数解析微分与 ACADO 仿射行组装

四旋翼在动基座系下的二阶动力学模型为：
$$\dot{\mathbf{p}} = \mathbf{v}, \qquad \dot{\mathbf{v}} = \boldsymbol{\Lambda}(\mathbf{u} - \mathbf{v}) + \mathbf{a}_{\mathrm{ni}} + \mathbf{d}_v$$
系统相对阶为 2。定义 Softmax 内部归一化权重：
$$w_m = \frac{\kappa_m \exp\left(-\frac{d_m^{\mathrm{cert}} - d_{\min}^{\mathrm{cert}}}{\sigma}\right)}{\sum_{j} \kappa_j \exp\left(-\frac{d_j^{\mathrm{cert}} - d_{\min}^{\mathrm{cert}}}{\sigma}\right)}, \quad \sum_{m\in\mathcal{I}} w_m = 1 \tag{10}$$

#### 1. 一阶 Lie 导数
$$\nabla_{\mathbf{p}} h_F = \sum_{m\in\mathcal{I}} w_m \mathbf{n}_m \in \mathbb{R}^3, \qquad \dot{h}_F = \sum_{m\in\mathcal{I}} w_m \mathbf{n}_m^T \mathbf{v}_{\mathrm{rel},m} \tag{11}$$
其中 $\mathbf{v}_{\mathrm{rel},m} = \mathbf{v} - \mathbf{v}_m$。

#### 2. 二阶 Lie 导数
体素相对漂移加速度：
$$\mathbf{a}_{\mathrm{drift},m} = -\boldsymbol{\Lambda}\mathbf{v} + \mathbf{a}_{\mathrm{ni}} - \mathbf{a}_m \tag{12}$$
单体素二阶微分（向心曲率项分母必须使用严格大于零的 $s_m$）：
$$\ddot{d}_m^{\mathrm{drift}} = \frac{\|\mathbf{v}_{\mathrm{rel},m}\|_2^2 - (\mathbf{n}_m^T \mathbf{v}_{\mathrm{rel},m})^2}{s_m} + \mathbf{n}_m^T \mathbf{a}_{\mathrm{drift},m} \tag{13}$$
全场聚合二阶漂移项（含对数求导产生的速度方差耗散项）：
$$L_f^2 h_F = \sum_{m\in\mathcal{I}} w_m \ddot{d}_m^{\mathrm{drift}} - \frac{1}{\sigma} \left[ \sum_{m\in\mathcal{I}} w_m (\mathbf{n}_m^T \mathbf{v}_{\mathrm{rel},m})^2 - (\dot{h}_F)^2 \right] \tag{14}$$

#### 3. ACADO 仿射安全约束行
定义复合李雅普诺夫判据 $\psi_1 = \dot{h}_F + \gamma_1 h_F$，$\psi_2 = \dot{\psi}_1 + \gamma_2 \psi_1 \ge 0$：
$$\boxed{\mathbf{A} \mathbf{u} \ge b} \tag{15}$$
其中：
$$\boxed{\mathbf{A} = \boldsymbol{\Lambda}^T \nabla_{\mathbf{p}} h_F \in \mathbb{R}^3} \tag{16}$$
$$\boxed{b = -\left[ L_f^2 h_F + (\gamma_1 + \gamma_2)\dot{h}_F + \gamma_1 \gamma_2 h_F \right] + \rho_{\mathrm{dyn}}} \tag{17}$$
式中 $\rho_{\mathrm{dyn}}$ 为扰动保守收缩项（工程中可设为 $D_{\max} \|\nabla_{\mathbf{p}} h_F\|$）。

---

## 3. 连续时间保证与感知截断的理论闭环

### 3.1 极限刹车距离物理不可达证明（合法化外部体素截断）

无需依靠平滑衰减核，而是依据四旋翼**最大飞行速度与物理减速度上限**证明截断的绝对安全性：
* 设四旋翼在世界系的最大空速为 $v_{W,\max}$，最大制动加速度为 $a_{\max}$，控制更新周期为 $\Delta t$；
* 无人机在当前帧时域内的极限紧急制动位移为：
  $$d_{\mathrm{brake}} = \frac{v_{W,\max}^2}{2 a_{\max}} + v_{W,\max} \Delta t \tag{18}$$
* **截断安全定理**：
  只要系统配置的感知认证半径满足：
  $$\boxed{R_{\mathrm{prune}} \ge d_{\mathrm{brake}} + r_{\mathrm{uav}} + r_m} \tag{19}$$
  则所有位于 $R_{\mathrm{prune}}$ 外部的未纳入体素，即便无人机以最大油门全速对冲，在当前控制周期内物理上也**绝对不可能发生碰撞**！因此排除它们不会破坏安全不变性。

### 3.2 分段连续时间（Piecewise $C^2$）前向不变性

* **帧内区间 $[t_k, t_{k+1})$**：
  在点云传感器更新的间隔期内，认证点集 $\mathcal{I}(t_k)$ 冻结。因为 $s_m$ 处处解析，LogSumExp 是解析可微的，$h_F(\mathbf{p})$ **在空间中严格满足 $C^\infty$**。连续时间比较引理在区间内 $100\%$ 成立：
  $$\psi_1(t) \ge \psi_1(t_k) e^{-\gamma_2 (t - t_k)} \ge 0 \implies h_F(t) \ge h_F(t_k) e^{-\gamma_1 (t - t_k)} \ge 0$$
* **刷新跃迁时刻 $t_k$**：
  新点云到达时，系统依据杂化系统（Hybrid Systems）的**切换相容性条件**无缝接续：只要新场满足 $h_{F,\mathrm{new}}(t_k^+) \ge 0$ 且 $\psi_{1,\mathrm{new}}(t_k^+) \ge 0$，系统在分段意义下严格保证全局前向不变性。

---

## 4. 求解失败备份控制规范（Certified Fallback）

### 4.1 传统“直接沿用上一次解”的致命隐患
当前代码中如果求解失败，直接返回上一周期的第 0 步指令 $\mathbf{u}_0^*(k-1)$：
1. **时间错位**：$\mathbf{u}_0^*$ 是上一时刻的指令，在当前时刻原本该执行的是 $\mathbf{u}_1^*(k-1)$，重复执行 $\mathbf{u}_0^*$ 会导致速度突变；
2. **约束失效**：当前速度 $\mathbf{v}_k$ 已经改变，$\boldsymbol{\Lambda}(\mathbf{u}_0^* - \mathbf{v}_k)$ 极易导致加速度超载或击穿当前最新的 CBF 屏障；
3. **连续失败失控**：连续数帧失败时，无人机会以常速闭眼盲飞撞墙。

### 4.2 阶梯式时域移位开环控制规范（Shifted Horizon Fallback）
* **瞬态失败（连续失败 $\le 2$ 周期，$< 40\text{ ms}$）**：
  依次提取上一周期已规划好的未来第 1 步或第 2 步控制量：
  $$\mathbf{u}_{\mathrm{fallback}}(t_k) = \mathbf{u}_1^*(k-1), \qquad \mathbf{u}_{\mathrm{fallback}}(t_{k+1}) = \mathbf{u}_2^*(k-1)$$
  *理论保证*：在上一周期优化求解中，$\mathbf{u}_1^*$ 本就是为 $t_k$ 规划的名义最优输入，天然满足动力学与安全约束，飞行动力学 $100\%$ 平滑。
* **持续故障（连续失败 $\ge 3$ 周期，$> 60\text{ ms}$）**：
  说明感知或优化陷入不可逆故障，立即触发**紧急安全制动**，下发零速度指令就地平稳悬停。

---

## 5. C++ 代码落地实现

### 5.1 重构 `field_hocbf.h`（真 3D 物理保守连续安全场）

替换 `marsim_ws/src/coni_mpc/include/coni_mpc/field_hocbf.h` 中的 `computeConstraint` 函数：

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace coni_mpc {
namespace field_hocbf {

struct Config {
  double sigma = 0.12;
  double d_safe = 0.48;
  double gamma1 = 1.2;
  double gamma2 = 4.5;
  double prune_radius = 2.0;
  double max_vertical_delta = 0.8;
  double min_occupancy = 0.15;
  std::size_t seed_count = 10;
  std::size_t max_count = 35;
  double separation = 0.15;
  double r_uav = 0.15;       // 无人机物理包络外接球半径 [m]
  double r_voxel = 0.13;     // 体素物理外接球半径 sqrt(3)/2 * 0.15 [m]
  double safety_margin = 0.20; // 最终物理安全净间距 d_margin [m]
};

struct KinematicPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  std::uint64_t source_id = 0;
};
using KinematicPoints =
    std::vector<KinematicPoint, Eigen::aligned_allocator<KinematicPoint>>;

struct Constraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
  double b = 0.0;
  double max_occupancy = 0.0;
  double h = std::numeric_limits<double>::quiet_NaN();
  double hdot = std::numeric_limits<double>::quiet_NaN();
  double lf2 = std::numeric_limits<double>::quiet_NaN();
  double d_min = std::numeric_limits<double>::quiet_NaN();
  double d_softmin = std::numeric_limits<double>::quiet_NaN();
  bool active = false;
};

inline Constraint computeConstraint(
    const KinematicPoints& points, 
    const Eigen::Vector3d& nominal_position,
    const Eigen::Vector3d& nominal_velocity,
    const Eigen::Vector3d& non_inertial_acceleration,
    const Config& config = Config(),
    const Eigen::Vector3d& lambda = Eigen::Vector3d::Constant(5.0)) {
  
  Constraint result;
  const double sigma = config.sigma;
  const double d_margin = config.safety_margin;
  const double gamma1 = config.gamma1;
  const double gamma2 = config.gamma2;
  const double eps = 1.0e-4;          // 正则化参数，平滑欧氏范数尖点
  const double r_uav = config.r_uav;
  const double r_voxel = config.r_voxel;

  if (points.empty() || !(sigma > 0.0) || !nominal_position.allFinite() ||
      !nominal_velocity.allFinite() || !non_inertial_acceleration.allFinite()) {
    result.active = false;
    return result;
  }

  struct VoxelTerm {
    Eigen::Vector3d normal;
    double kappa;
    double d_cert;
    double hdot_m;
    double hddot_drift_m;
  };

  std::vector<VoxelTerm> terms;
  terms.reserve(points.size());
  double d_min_cert = std::numeric_limits<double>::infinity();
  result.max_occupancy = 0.0;

  for (const auto& point : points) {
    if (!(point.occupancy > 0.0) || !std::isfinite(point.occupancy)) continue;

    VoxelTerm item;
    // 证书权重保证 kappa >= 1.0，防止低概率稀释物理实体
    item.kappa = 1.0 + 1.0 * point.occupancy; 
    result.max_occupancy = std::max(result.max_occupancy, point.occupancy);

    // 1. 真 3D 相对位移向量（彻底删除 .z() = 0.0）
    const Eigen::Vector3d delta_p = nominal_position - point.position;
    const double s_m = std::sqrt(delta_p.squaredNorm() + eps * eps);
    item.normal = delta_p / s_m; // 严格光滑的 3D 外法向 (范数 < 1)

    // 2. 严格几何物理保守净间隙 (减 eps 抵消高估，扣除双方外接球)
    item.d_cert = s_m - eps - r_uav - r_voxel;

    // 3. 3D 相对运动学与动基座动力学
    const Eigen::Vector3d v_r = nominal_velocity - point.velocity;
    const Eigen::Vector3d a_drift = non_inertial_acceleration -
                                    lambda.cwiseProduct(nominal_velocity) -
                                    point.acceleration;

    item.hdot_m = item.normal.dot(v_r);
    // 曲率项分母使用严格正数 s_m，避免奇异性
    const double v_tan_sq = std::max(0.0, v_r.squaredNorm() - item.hdot_m * item.hdot_m);
    item.hddot_drift_m = v_tan_sq / s_m + item.normal.dot(a_drift);

    d_min_cert = std::min(d_min_cert, item.d_cert);
    terms.push_back(item);
  }

  if (terms.empty() || !std::isfinite(d_min_cert)) {
    result.active = false;
    return result;
  }

  // 4. 移位 LogSumExp 计算 (绝不除以 sum_occupancy)
  const double inv_sigma = 1.0 / sigma;
  double shifted_partition_sum = 0.0;
  std::vector<double> unnorm_weights(terms.size(), 0.0);

  for (std::size_t m = 0; m < terms.size(); ++m) {
    const double exp_val = std::exp(-(terms[m].d_cert - d_min_cert) * inv_sigma);
    unnorm_weights[m] = terms[m].kappa * exp_val;
    shifted_partition_sum += unnorm_weights[m];
  }

  if (!(shifted_partition_sum > 0.0) || !std::isfinite(shifted_partition_sum)) {
    result.active = false;
    return result;
  }

  // 严格保守平滑下界：h_F <= d_min_cert - d_margin <= D_phys - d_margin
  const double d_softmin = d_min_cert - sigma * std::log(shifted_partition_sum);
  result.h = d_softmin - d_margin;
  result.d_min = d_min_cert;
  result.d_softmin = d_softmin;

  // 5. 聚合梯度与二阶导数 (Softmax 权重归一化)
  Eigen::Vector3d grad_h = Eigen::Vector3d::Zero();
  double hdot = 0.0;
  double hddot_drift_mean = 0.0;

  for (std::size_t m = 0; m < terms.size(); ++m) {
    const double w_m = unnorm_weights[m] / shifted_partition_sum;
    grad_h += w_m * terms[m].normal;
    hdot += w_m * terms[m].hdot_m;
    hddot_drift_mean += w_m * terms[m].hddot_drift_m;
  }

  // Softmax 导数产生的速度方差耗散修正项
  double hdot_variance = 0.0;
  for (std::size_t m = 0; m < terms.size(); ++m) {
    const double dhdot = terms[m].hdot_m - hdot;
    const double w_m = unnorm_weights[m] / shifted_partition_sum;
    hdot_variance += w_m * dhdot * dhdot;
  }

  // 组装最终送给 ACADO 的仿射参数
  result.lf2 = hddot_drift_mean - inv_sigma * hdot_variance;
  result.hdot = hdot;
  result.A = lambda.cwiseProduct(grad_h); // 3D 输入矩阵 Lambda * grad_h
  result.b = -(result.lf2 + (gamma1 + gamma2) * hdot + gamma1 * gamma2 * result.h);
  result.active = result.A.allFinite() && std::isfinite(result.b) && std::isfinite(result.h);

  if (!result.active) {
    result.A.setZero();
    result.b = 0.0;
  }
  return result;
}

}  // namespace field_hocbf
}  // namespace coni_mpc
```

---

### 5.2 改造 `mpc_controller.cpp`（时域移位开环 Fallback）

#### 1. 在头文件 `include/acado_mpc/mpc_controller.h` 第 240 行添加计数器：
```cpp
  acado_mpc_common::ControlCommand last_valid_control_;
  bool has_last_valid_control_;
  // 新增：连续求解失败计数器
  std::size_t consecutive_failures_ = 0; 
```

#### 2. 在源文件 `src/acado_mpc/mpc_controller.cpp` 中重构 `makeFailureFallback` 与成功重置逻辑：

```cpp
// 1. 在求解成功即将返回前（第 512 行附近）：
last_valid_control_ = command;
has_last_valid_control_ = true;
consecutive_failures_ = 0; // 成功则计数归零
return command;

// 2. 重写 makeFailureFallback（第 209 行附近）：
auto makeFailureFallback = [this, call_time, field_active](const char* reason) {
  last_command_used_fallback_ = true;
  last_fallback_had_previous_valid_ = has_last_valid_control_;
  consecutive_failures_++;

  // 阶梯 A：瞬态短时失败（1~2 周期内，即 40ms 内）-> 执行上一周期预测的第 1 或第 2 步
  if (has_last_valid_control_ && 
      consecutive_failures_ <= 2 && 
      consecutive_failures_ < kSamples) {
    ROS_WARN_THROTTLE(
        0.5, "[MpcController] Solve failed (%s), applying shifted horizon step %zu", 
        reason, consecutive_failures_);

    // 提取上一周期已规划好的未来输入与状态
    const auto shifted_input = predicted_inputs_.col(consecutive_failures_);
    const auto shifted_state = predicted_states_.col(consecutive_failures_);

    // 执行更新与幅值安全保护
    return updateControlCommand(shifted_state, shifted_input, call_time, field_active);
  }

  // 阶梯 B：连续多次严重失败（>= 3 周期，即 > 60ms）-> 就地紧急制动悬停
  ROS_ERROR_THROTTLE(
      0.5, "[MpcController] PERSISTENT SOLVER FAILURE (%zu steps after %s) -> EMERGENCY HOVER!",
      consecutive_failures_, reason);

  acado_mpc_common::ControlCommand emergency_brake;
  emergency_brake.velocity_cmd.setZero();
  emergency_brake.yaw_rate = 0.0;
  emergency_brake.slack = 0.0;
  return emergency_brake;
};
```

---

## 6. 参数配置规范（YAML 映射）

在 [`parameters/num_sim_non_one_point.yaml`](file:///home/jjm/桌面/DSP/marsim_ws/src/coni_mpc/parameters/num_sim_non_one_point.yaml) 中核对配置项：

```yaml
# 物理限制解耦配置
max_v_xy: 20.0        # ACADO 控制输入 u 的数值松弛边界（防数值发散，不做物理截断）
max_v_z: 5.0
max_yaw_rate: 3.14

# 真实物理约束（对应四旋翼硬件极限）
a_max_xy: 6.0         # 最大水平合加速度 [m/s^2]，对应最大倾角 ~31 度
a_max_z: 4.0          # 最大垂直加速度 [m/s^2]

cbf:
  enabled: true
  use_field_hocbf: true
  field_sigma: 0.12     # 平滑因子 sigma
  field_d_safe: 0.20    # 物理安全裕度 d_margin [m]
  field_prune_radius: 2.0  # 感知球半径 R_prune >= d_brake + r_uav + r_m
  alpha1: 2.0           # HOCBF 极点 gamma1
  alpha2: 5.0           # HOCBF 极点 gamma2
  slack_max: 0.0        # 理论认证模式强制硬约束 slack = 0
```

---

## 7. 审稿人答辩核心论点（FAQ）

* **Q1：为什么你们的场没有对体素做平滑空间截断，依然能保证连续控制？**
  * **答**：我们在单个控制周期内将点集冻结，场函数由光滑解析函数解析复合而成，空间处处 $C^\infty$ 无穷阶可微。我们通过飞行器最大制动距离不等式 $R_{\mathrm{prune}} \ge \frac{v_{\max}^2}{2a_{\max}} + r_{\mathrm{uav}} + r_m$ 证明了视距外的障碍物在当前周期物理不可达。跨周期的更新作为杂化系统的分段前向不变性（Piecewise Forward Invariance）处理，比人为添加截断衰减核更具物理真实性。
* **Q2：加权 LogSumExp 是如何做到绝对无碰撞保证的？**
  * **答**：我们通过严格代数不等式证明了 $h_F(\mathbf{p}) \le d_{\min}^{\mathrm{cert}} - d_{\mathrm{margin}} \le D_{\mathrm{phys}} - d_{\mathrm{margin}}$。本算子绝不除以占据率之和，且保持权重 $\kappa_m \ge 1$，彻底杜绝了将真实危险虚假低估为安全的数学漏洞。

