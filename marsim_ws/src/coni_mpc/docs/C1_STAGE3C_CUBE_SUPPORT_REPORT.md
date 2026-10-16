# C1 阶段 3C：纯 P2G 立方体支持域上界审计

本阶段只新增独立 Python 参考实现、测试和报告，没有修改阶段 3A、ROS、DSP 发布链路、HOCBF、MPC、ACADO 或在线风险接口。

- 参考实现：[`stage3c_cube_support_experiment.py`](../scripts/stage3c_cube_support_experiment.py)
- 测试：[`stage3c_cube_support_test.py`](../test/stage3c_cube_support_test.py)

## 1. 现有实现审计

`dsp_particle_to_grid.cpp` 使用固定节点

```text
c_j = origin + Delta * (j_x,j_y,j_z)
```

每个轴使用 `floor(p/Delta)-1,...,floor(p/Delta)+2` 的四个 cubic cardinal B-spline 权重，三维权重是三个一维权重的乘积。权重非负；完整支持域内共有最多 64 个节点。对一个节点 `c_j`，非零权重的粒子坐标满足

```text
|p_x-c_jx| <= 2 Delta,
|p_y-c_jy| <= 2 Delta,
|p_z-c_jz| <= 2 Delta.
```

所以真实支持域可以安全写成闭立方体

```text
Q_j = c_j + [-2 Delta,2 Delta]^3.
```

当前 P2G 对正权重支持越过有限网格直接抛出 `std::out_of_range`，不会静默裁剪或归一化剩余权重。因而现有实现只有在所有正权重支持节点均在网格内时才具有质量守恒和完整矩恒等式。

`conservative_grid_risk.cpp` 的阶段 3A 上界使用

```text
rho = 2 sqrt(3) Delta,
d_ball(q,c_j) = max(||q-c_j||-rho, 0),
R_3A = sum_j M_j kappa(d_ball(q,c_j)).
```

这里 `kappa` 是现有物理半径 `r` 加过渡宽度 `delta_sm` 的五次 smootherstep，单调非增，在平台和支撑外部导数为零。阶段 3B 报告已证明完整支持域的矩性质，但减号 Taylor 修正已经被反例否定；本阶段不使用 Taylor 修正。

## 2. 纯 P2G 的立方体候选

定义点到支持立方体的距离

```text
d_cube(q,c_j) = sqrt(sum_k max(|q_k-c_jk|-2 Delta, 0)^2)
```

并定义

```text
R_cube(q) = sum_j M_j kappa(d_cube(q,c_j)).
```

这只需要节点质量 `M_j` 和网格分辨率，不需要粒子位置、速度或额外接口。

### 完整支持域中的保守性证明

令 `q_i>=0` 为粒子风险权重，且 `N_ij>=0`、`sum_j N_ij=1`。对所有 `N_ij>0`，有 `p_i in Q_j`，因此

```text
||q-p_i|| >= d_cube(q,c_j).
```

由于 `kappa` 单调非增，逐项得到

```text
kappa(||q-p_i||) <= kappa(d_cube(q,c_j)).
```

乘以 `q_i N_ij` 并求和：

```text
R_original
 = sum_i q_i sum_j N_ij kappa(||q-p_i||)
 <= sum_i q_i sum_j N_ij kappa(d_cube(q,c_j))
 = sum_j M_j kappa(d_cube(q,c_j))
 = R_cube.
```

这个证明不需要二阶矩，只需要非负权重、分割统一、完整支持域以及核的单调性。

### 与阶段 3A 的逐节点比较

外接球半径 `rho=2 sqrt(3) Delta` 满足 `Q_j subset B(c_j,rho)`，所以

```text
d_cube(q,c_j) >= d_ball(q,c_j).
```

再次利用 `kappa` 单调非增，有

```text
kappa(d_cube(q,c_j)) <= kappa(d_ball(q,c_j)).
```

因此在同一完整投影上严格得到

```text
R_original <= R_cube <= R_3A.
```

这是一个比 Taylor 加号修正更直接的值域证明，也说明立方体候选确实能消除外接球在角方向上的一部分膨胀。

## 3. 边界和局部裁剪不能套用上述证明

### 地图边界

当前 C++ P2G 在支持越界时直接抛异常，因此边界场景没有合法的 `Projection`，上述上界只能声明为“不适用”，不能把裁剪后的结果当作已证明上界。

独立参考实现显式模拟裁剪：`Delta=1`、8³ 网格、粒子 `p=(0.2,0.3,0.4)`、查询点取粒子位置、`r=0.5`、`delta_sm=0.5`。结果为

```text
input mass   = 1.0000000000
clipped mass = 0.8313326062
R_original   = 1.0000000000
R_cube       = 0.8313326062
R_3A         = 0.8313326062
```

两个网格量都低于原始风险。该反例来自丢失支持质量，而不是立方体几何错误。

### 查询局部网格裁剪

即使 P2G 完整，如果风险求和时只保留查询附近的一小部分节点，也会破坏 `sum_j N_ij=1` 的求和。单粒子 `p=(3.37,3.23,3.19)` 的完整投影有 64 个节点；只保留一个节点时，`R_original=1`，而裁剪后的 `R_cube=0.0002808634`。因此候选必须使用完整 P2G 节点集合，不能把查询窗口裁剪当作无害优化。

## 4. 数值比较

独立测试覆盖单粒子、5 粒子稀疏分布、80 粒子密集分布、过渡区域和多个查询位置。完整支持域下未发现违反

```text
R_original <= R_cube <= R_3A
```

的样本；最小数值裕度仅为约 `-1.1e-16` 的浮点舍入量。

一组随机搜索的最大保守量如下（`Delta=0.5,r=1,delta_sm=0.5`）：

| 场景 | 节点数 | max(`R_cube-R_original`) | max(`R_3A-R_cube`) |
|---|---:|---:|---:|
| 单粒子 | 64 | 0.9857 | 0.5609 |
| 稀疏 5 粒子 | 320 | 0.9718 | 0.3775 |
| 密集 80 粒子 | 777 | 32.5189 | 13.2071 |

参数扫描也保持两侧不等式。代表性的 `max(R_3A-R_cube)` 为：

| `Delta` | `delta_sm=0.25` | `delta_sm=0.5` | `delta_sm=1.0` |
|---:|---:|---:|---:|
| 1.0 | 0.7439 | 0.5755 | 0.2818 |
| 0.5 | 0.5688 | 0.4228 | 0.4168 |
| 0.25 | 0.5046 | 0.3709 | 0.1901 |

这些数值说明立方体候选在值域上确实比阶段 3A 紧，但这还不等于可以接入控制器，因为导数正则性失败。

## 5. 梯度、Hessian 与 Field-HOCBF 兼容性

在固定 active set（哪些坐标超出立方体）且过渡区内，令 `y_k=max(|q_k-c_jk|-2Delta,0)`，`d=||y||`。参考实现使用

```text
grad d = sign(q-c) .* y / d,
H_d = diag(active)/d - (grad d)(grad d)^T/d,
grad(kappa(d)) = kappa'(d) grad d,
H(kappa(d)) = kappa''(d) grad d grad d^T + kappa'(d) H_d.
```

远离 active-set 切换面时，解析梯度/Hessian 与中心差分一致，测试误差分别低于 `2e-6` 和 `2e-5`。

但是，`d_cube` 在立方体面、棱、角的 active-set 切换处通常只有 C¹。物理平台会掩盖 `d<=r` 的切换，但不能掩盖“一个坐标刚越过立方体面、另一个坐标已经处于 smootherstep 过渡区”的情况。

可复现例子：`Delta=1`、`r=0.5`、`delta_sm=0.75`、`c=(0,0,0)`，在 `q=(2,2.8,0)` 处横穿 `x=2` 立方体面。梯度左右连续，但

```text
H_xx(left)  = 0
H_xx(right) = -2.8800000005
```

Hessian 存在明显跳变。因此 `R_cube` 是分片二阶可导、整体至多 C¹，不能满足需要全局连续 Hessian 或更高阶 Lie 导数的 Field-HOCBF 数学接口。对距离函数做软化会改变逐点上确界，必须重新证明保守性，不能直接平滑后接入。

## 6. 计算负载

独立 Python 参考实现按 21 个预测阶段测量 P2G、立方体风险和阶段 3A 球风险，`Delta=0.5` 的一次参考结果为：

| 粒子数 | 最大非零 P2G 节点数 | 总耗时 | 每阶段 |
|---:|---:|---:|---:|
| 256 | 13,856 | 3.19 s | 152 ms |
| 1,024 | 34,757 | 8.08 s | 385 ms |
| 2,048 | 44,887 | 10.66 s | 507 ms |

这是未优化的 Python 参考实现，不能当作 C++ 实时性能结论；它表明逐节点立方体距离还需要遍历每个 P2G 节点，但不会增加在线接口允许的节点数据维度。

## 7. 结论

在**完整 P2G 支持域、未裁剪节点集合、非负权重、单调 smootherstep 核**的前提下，立方体候选严格满足

```text
R_original <= R_cube <= R_3A.
```

因此，从风险值紧致性角度，它比阶段 3A 更有希望；证明不依赖阶段 3B 的 Taylor 修正。

但该候选的全局 Hessian 不连续，且边界/局部节点裁剪会直接破坏保守性。按照当前 Field-HOCBF 的导数要求和本阶段“不修改在线链路”的限制，**现在不建议接入**。阶段 3A 应继续冻结。

若要继续研究，下一步必须先选择一种能够保持逐点上界的 C² 支持域近似，并重新证明

```text
R_original <= R_new <= R_3A
```

以及全局梯度/Hessian 连续性；不能仅对 `d_cube` 做经验平滑。
