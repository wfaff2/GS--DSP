# C1 阶段 3D：四次超椭球支持域审计

本阶段只新增独立 Python 参考实现和测试，没有修改阶段 3A、ROS、DSP 发布链路、Field-HOCBF、MPC、ACADO 或在线风险接口。

- 参考实现：[stage3d_superellipsoid_experiment.py](../scripts/stage3d_superellipsoid_experiment.py)
- 测试：[stage3d_superellipsoid_test.py](../test/stage3d_superellipsoid_test.py)

## 1. 审计对象和假设

当前 P2G 的节点为 `c_j=origin+Delta*j`，每个轴使用四个 cubic cardinal B-spline 权重，三维权重为一维权重乘积。权重非负；完整支持域中，若 `N_ij>0`，则

审计依据为现有 [dsp_particle_to_grid.cpp](../src/dsp_particle_to_grid.cpp)、[dsp_particle_to_grid.h](../include/coni_mpc/dsp_particle_to_grid.h) 和 [conservative_grid_risk.cpp](../src/conservative_grid_risk.cpp)。

```text
|p_i,k-c_j,k| <= 2 Delta,  k=1,2,3.
```

因此节点 `j` 的真实支持立方体是

```text
Q_j = c_j + [-a,a]^3,       a=2 Delta.
```

阶段 3A 使用半径 `rho=sqrt(3)*a=2 sqrt(3) Delta` 的外接球。风险核是物理半径 `r>0`、过渡宽度 `delta_sm>0` 的五次 smootherstep，记为 `kappa(d)`；它非负、单调非增，并在平台和支撑外部具有零的一阶、二阶导数。

以下证明假定：

1. 所有正权重的 64 个支持节点均存在；当前 C++ P2G 对越界支持直接抛出异常，不把边界质量静默裁掉。
2. 节点质量 `M_j` 非负，且 `M_j=sum_i q_i N_ij`。
3. 风险求和使用完整 P2G 节点集合，没有按查询点再裁剪。

这些条件是值域证明的必要条件，不是实现细节上的可选项。

## 2. 候选四次超椭球

对每个节点定义

```text
S_j = c_j + S,
S = { z : z_1^4+z_2^4+z_3^4 <= 3 a^4 }.
```

新风险使用该集合的**精确欧氏距离**：

```text
R_super(q) = sum_j M_j kappa(dist(q,S_j)).
```

实现没有使用 `||q-c_j||` 与某个经验半径的差值来代替 `dist(q,S_j)`，而是求解最近点投影的 KKT 方程。

### 2.1 几何包含关系

若 `z in Q_j`，则每个 `|z_k|<=a`，所以 `sum z_k^4<=3a^4`，从而 `Q_j subset S_j`。

对 `z in S`，范数不等式给出

```text
||z||_2 <= 3^(1/4) ||z||_4 <= 3^(1/4) (3a^4)^(1/4) = sqrt(3) a.
```

因此 `S_j subset B(c_j,rho)`，其中 `rho=sqrt(3)a`。数值扫描 10,000 个点也没有发现包含关系违例；解析关系才是证明依据。

### 2.2 精确最近点投影、唯一性和光滑性

令 `y=q-c_j`。在 `y` 位于 `S` 外部时，最近点 `x` 满足

```text
y = x + 4 lambda x^3,
sum_k x_k^4 = 3a^4,
lambda > 0.
```

`x^3` 按坐标计算。固定 `lambda>=0` 时，标量函数 `x+4 lambda x^3` 严格递增，因此坐标解唯一。随着 `lambda` 增大，约束函数 `sum x_k^4` 严格下降，从 `lambda=0` 的外部值下降到 0，故存在且仅存在一个满足等式的 `lambda`。这也与 `S` 的闭凸性给出的唯一欧氏投影一致。

每个坐标使用三次方程的双曲正弦闭式解

```text
x_k = 2/sqrt(12 lambda) * sinh(asinh(1.5 sqrt(12 lambda) y_k)/3).
```

对 KKT 系统定义

```text
A = I + 12 lambda diag(x^2),  b=4x^3.
```

在边界上 `b` 不为零，`A` 正定；KKT 块 Jacobian 的 Schur 补为

```text
-b^T A^(-1) b < 0.
```

所以隐函数定理保证 `x(q)`、`lambda(q)` 以及外部区域的 `dist(q,S)` 是 `C^infinity`。这一步排除了阶段 3C 立方体 active-set 切换带来的 Hessian 跳变。这里的结论针对精确投影；普通范数差值不具备该证明。

### 2.3 导数公式

令

```text
P = A^(-1) - A^(-1)b b^T A^(-1)/(b^T A^(-1)b),
n = b/||b||.
```

则

```text
Dx/Dq = P,
H_d = (I-n n^T) diag(12x^2) P / ||b||,
grad d = n.
```

实现对 `H_d` 做对称化以消除浮点反对称噪声。过渡区中的风险项满足

```text
grad kappa(d) = kappa'(d) n,
H kappa(d) = kappa''(d) n n^T + kappa'(d) H_d.
```

距离函数在集合边界本身一般不能直接当作全局 `C^2` 函数；但是 `r>0` 时，`d<=r` 的整个平台把该边界包在常值邻域内。`d=r` 和 `d=r+delta_sm` 处，五次 smootherstep 的 `kappa'`、`kappa''` 均为 0，故每个节点风险项及其有限和具有全局 `C^2` 空间正则性。若 `r=0` 或 `delta_sm=0`，这个结论不成立；这也是本审计明确保留的物理半径和过渡宽度条件。

## 3. 风险值夹逼证明

记点到集合的距离为 `d_A(q)`。由 `Q_j subset S_j subset B_j`，距离的包含反序给出

```text
d_Bj(q) <= d_Sj(q) <= d_Qj(q).
```

由于 `kappa` 单调非增，逐节点有

```text
kappa(d_Qj) <= kappa(d_Sj) <= kappa(d_Bj).
```

另一方面，对每个 `N_ij>0`，粒子 `p_i in Q_j`，因此

```text
d_Qj(q) <= ||q-p_i||,
kappa(||q-p_i||) <= kappa(d_Qj(q)).
```

乘以非负 `q_i N_ij` 并对 `i,j` 求和，使用完整支持域的 `M_j=sum_i q_i N_ij`，得到严格的值域链

```text
R_original(q) <= R_cube(q) <= R_super(q) <= R_3A(q).
```

因此四次超椭球候选满足本阶段所要求的两端不等式 `R_original<=R_super<=R_3A`。它比阶段 3C 立方体上界更保守，这是集合大小关系决定的，不能把它表述成 3C 的改进。

如果边界裁剪或查询局部裁剪使 `M_j` 不再等于完整支持求和，上述证明立即失效。独立反例为 `Delta=1`、8^3 网格、`p=(0.2,0.3,0.4)`：裁剪后质量为 `0.8313326062`，而粒子位置处的原始风险为 `1.0`，超椭球风险也只有 `0.8313326062`。这不是超椭球几何的反例，而是 P2G 信息丢失的反例。

## 4. 数值审计

命令：

```bash
PYTHONPATH=marsim_ws/src/coni_mpc/scripts \
python3 -m unittest -v marsim_ws/src/coni_mpc/test/stage3d_superellipsoid_test.py
```

9/9 项测试通过，覆盖几何包含、KKT 投影、批量与标量投影一致性、单粒子/稀疏/密集完整支持风险夹逼、核过渡区、有限差分导数和边界裁剪反例。

一次固定种子扫描（`Delta=0.5, r=1, delta_sm=0.5`）得到：

| 场景 | P2G 节点数 | min(`R_cube-R_original`) | min(`R_super-R_cube`) | min(`R_3A-R_super`) | max(`R_cube-R_original`) |
|---|---:|---:|---:|---:|---:|
| 单粒子 | 64 | `2.2e-16` | `0` | `0` | `0.9664` |
| 稀疏 6 粒子 | 348 | `-1.1e-16` | `0` | `0` | `2.1719` |
| 密集 40 粒子 | 551 | `0.2060` | `0.0791` | `0.0165` | `17.6866` |

负值均为浮点舍入量。投影扫描 500 个外部查询的最大约束残差为 `7.1e-14`，最大 KKT 残差为 `5.8e-15`；风险扫描中抽样节点的最大约束残差为 `4.9e-15`。

在密集 40 粒子这组查询中，`mean(R_super-R_original)=12.8867`，`mean(R_3A-R_super)=1.1266`，所以四次超椭球相对 3A 的平均过度量减少约 `8.0%`；立方体相对 3A 的减少约 `17.8%`，但立方体的 Hessian 正则性仍有阶段 3C 已知跳变。单粒子和稀疏场景的减少分别约 `0.4%` 和 `10.0%`，这些百分比只是固定种子、固定查询集合的比较，不能当作全局性能保证。

在 `Delta in {0.25,0.5,1.0}` 和 `delta_sm in {0.25,0.5,1.0}` 的 9 组扫描中，没有值域违例。较大的 `R_3A-R_super` 约为 `0.64`（该小规模扫描的绝对量）；这只能说明参考场景中的相对紧致性，不能替代全状态证明。理论上的保守链已由第 3 节给出。

固定查询点的有限差分导数审计（`Delta=1,r=0.5,delta_sm=0.8`）为：

```text
gradient error = 1.58e-9
Hessian error  = 8.59e-9
constraint residual = -2.13e-14
KKT residual        = 4.71e-16
```

100 个随机外部查询相对于 96 次二分参考解的最大距离误差为：24 次 `2.70e-7`、32 次 `2.57e-9`、48 次 `1.23e-14`、64 次小于机器精度。这验证了参考实现的数值收敛，但不是对未来实时求解器误差的性能承诺。

## 5. 计算负载和未解决事项

独立 Python 参考实现使用精确标量投影，在 21 个预测阶段上测得：

| 粒子数 | 最大 P2G 节点数 | 总耗时 | 每阶段 |
|---:|---:|---:|---:|
| 16 | 942 | 5.03 s | 239.5 ms |
| 64 | 3307 | 17.04 s | 811.3 ms |
| 256 | 7178 | 35.19 s | 1675.7 ms |

这是未优化的 Python 精确投影参考成本，不是 C++ 在线性能结论。它显示每节点 KKT 投影的计算量明显高于简单球距离；尚未完成缓存、向量化精确投影或适合 ACADO 的固定维度实现。

已经严格证明的结论是：完整支持、非负质量、非裁剪节点集合和 `r>0,delta_sm>0` 下的集合包含、投影唯一性、外部距离光滑性、核拼接 `C^2` 以及风险值夹逼。数值测试只用于验证实现残差、导数公式和场景覆盖，不能把有限扫描升级为全局证明。

仍未证明或未实现的内容包括：含地图边界的在线质量保持策略、节点质量随时间变化时的完整混合时空导数、精确投影的实时计算上界，以及当前 Field-HOCBF 所需固定接口下的闭环安全性。候选因此不应接入在线链路；阶段 3A 继续作为已验证基线。若后续继续，优先级应是降低精确投影成本并审计时空导数，而不是修改 HOCBF 或 MPC。

