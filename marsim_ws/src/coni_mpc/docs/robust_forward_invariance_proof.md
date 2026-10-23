# 动基座非惯性系下 UAV 鲁棒前向不变性：严格修订版（连续时间、条件性定理）

> **研究定位**：本文证明，在真实障碍物获得确定性几何覆盖、UGV 非惯性预测误差有已知有效界、硬鲁棒 HOCBF 约束持续得到执行的条件下，UAV 的物理安全间隙保持非负。**本文不声称已经证明输入受限下的递推可行性，也不直接声称 10 Hz 离散控制实现获得了连续时间保证。**
>
> **代码对应**：以 `wfaff2/GS--DSP` 仓库 `1016` 分支中的 `marsim_ws/src/coni_mpc/acado_model/quadrotor_model_thrustrates.cpp` 与 `marsim_ws/src/coni_mpc/include/coni_mpc/field_hocbf.h` 作为已有模型参考。下面的认证距离场和收缩项是建议设计，**不是仓库中已经完成的实现**。

---

## 1. 相对动力学与适用范围

令 $\mathcal F_B$ 为固连 UGV 的非惯性动基座，UAV 状态为 $\mathbf x=(\mathbf p^\top,\mathbf v^\top)^\top$，速度指令 $\mathbf u\in\mathbb R^3$ 在该模型中表示为动基座坐标系下的相对速度指令。预测模型为

$$
\boxed{\dot{\mathbf p}=\mathbf v,\qquad
\dot{\mathbf v}=\boldsymbol\Lambda(\mathbf u-\mathbf v)+\mathbf a_{\mathrm{ni}}+\mathbf d_v.}
\tag{1}
$$

其中 $\boldsymbol\Lambda=\operatorname{diag}(\lambda_x,\lambda_y,\lambda_z)$、$\lambda_i>0$；$\mathbf d_v$ 表示若考虑真实底层执行的模型残差（若只审查理想预测模型，则取零）。非惯性项按源码定义：

$$
\boxed{
\mathbf a_{\mathrm{ni}}
=-\mathbf a_{\mathrm{car}}
-2\boldsymbol\omega\times\mathbf v
-\boldsymbol\beta\times\mathbf p
-\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf p).
}
\tag{2}
$$

其中 $\mathbf a_{\mathrm{car}}=\mathbf R^\top\mathbf a_g$、$\boldsymbol\beta=\dot{\boldsymbol\omega}$，各向量与障碍物位置、速度、加速度必须统一使用动基座坐标系。

**重要边界**：式 (1) 与当前 ACADO 预测模型相容，但实际下层速度环是否满足该模型，需要另行辨识或将残差纳入 $\mathbf d_v$ 的已知界。

## 2. 非惯性因果预测误差：保留完整二阶项

定义预测参数与实际参数之差：

$$
\tilde{\mathbf a}_{\mathrm{car}}=\mathbf a_{\mathrm{car}}-\hat{\mathbf a}_{\mathrm{car}},\quad
\tilde{\boldsymbol\omega}=\boldsymbol\omega-\hat{\boldsymbol\omega},\quad
\tilde{\boldsymbol\beta}=\boldsymbol\beta-\hat{\boldsymbol\beta}.
$$

**假设 A1（因果有界预测误差）**：在所考虑的预测承诺有效时间区间内，真实误差满足

$$
\|\tilde{\mathbf a}_{\mathrm{car}}\|_2\le\delta_a,\qquad
\|\tilde{\boldsymbol\omega}\|_2\le\delta_\omega,\qquad
\|\tilde{\boldsymbol\beta}\|_2\le\delta_\beta.
\tag{3}
$$

其中各上界可以是经过认证的时变值，但必须覆盖**实际**误差，不能只把训练集或仿真样本中的最大误差当作绝对上界。

从式 (2) 做精确展开，得到

$$
\boxed{
\begin{aligned}
\Delta\mathbf a_{\mathrm{ni}}
={}&-\tilde{\mathbf a}_{\mathrm{car}}
+\mathbf M_\omega\tilde{\boldsymbol\omega}
+[\mathbf p]_\times\tilde{\boldsymbol\beta}\\
&-\tilde{\boldsymbol\omega}\times
(\tilde{\boldsymbol\omega}\times\mathbf p),
\end{aligned}}
\tag{4}
$$

其中

$$
\boxed{
\mathbf M_\omega
=2[\mathbf v]_\times
+[\hat{\boldsymbol\omega}\times\mathbf p]_\times
+[\hat{\boldsymbol\omega}]_\times[\mathbf p]_\times.}
\tag{5}
$$

特别地，最后一项是**角速度预测误差的二阶项**。若将其丢弃，则只能得到一阶近似，而不是严格的扰动模型。

设可认证的额外加速度残差满足 $\|\mathbf d_v\|_2\le\delta_v$。对于给定的场梯度 $\mathbf g$，定义保守收缩：

$$
\boxed{
\begin{aligned}
\rho_{\mathrm{dyn}}={}&
(\delta_a+\delta_v)\|\mathbf g\|_2
+\delta_\omega\|\mathbf M_\omega^\top\mathbf g\|_2\\
&+\delta_\beta\|[\mathbf p]_\times^\top\mathbf g\|_2
+\delta_\omega^2\|\mathbf p\|_2\|\mathbf g\|_2.
\end{aligned}}
\tag{6}
$$

由 Cauchy–Schwarz 和叉乘范数不等式，严格有

$$
\boxed{-\mathbf g^\top(\Delta\mathbf a_{\mathrm{ni}}+\mathbf d_v)\le\rho_{\mathrm{dyn}}.}
\tag{7}
$$

式 (6) 是**保守上界**，不应写成对耦合误差的“精确上确界”。如障碍物速度、加速度及状态估计也存在误差，必须单独验证其对 $\dot h,\ddot h$ 的影响，并加入相应有效上界；本文下面首先使用障碍物轨迹已获认证的理想条件。

---

## 3. 三维物理几何间隙与认证安全场

### 3.1 几何覆盖条件与体素外接球

设 UAV 真实机体被半径为 $r_{\mathrm{uav}}>0$ 的球体保守包络覆盖；第 $m$ 个体素中心在动基座内为 $\mathbf c_m(t)\in\mathbb R^3$，相应整个体素由球 $B(\mathbf c_m,r_m)$ 覆盖。

对于边长为 $\ell_m$ 的正方体体素，其外接球**半径**为

$$
\boxed{r_m=\frac{\sqrt{3}}{2}\ell_m,}
\tag{8}
$$

而不是体素的完整空间对角线长度。对于非立方体体素，取半对角线长。

**假设 A2（确定性障碍物覆盖）**：在安全证书有效期内，所有需要避免的真实障碍物实体被已认证的有限体素球并集覆盖：

$$
\boxed{\mathcal O_{\mathrm{real}}(t)
\subseteq \bigcup_{m\in\mathcal I(t)}B(\mathbf c_m(t),r_m),\quad
\mathcal I(t)\ne\varnothing.}
\tag{9}
$$

这里 $\mathcal I(t)$ 是**安全认证集合**，不等同于按照占据概率/距离筛选后仅用于加速控制的任意点集。过滤、未观测区和概率阈值均不得破坏 (9)；如果无法保证 (9)，结论仅针对已覆盖的障碍物，不是全局真实无碰撞。

### 3.2 纠正正则化引起的距离高估

令

$$
\mathbf q_m=\mathbf p-\mathbf c_m(t),\qquad
s_m=\sqrt{\|\mathbf q_m\|_2^2+\epsilon^2},\quad\epsilon>0.
$$

采用经过**向下修正**的光滑保守间隙

$$
\boxed{d_m^{\mathrm{cert}}=s_m-\epsilon-r_{\mathrm{uav}}-r_m.}
\tag{10}
$$

因 $s_m-\epsilon\le\|\mathbf q_m\|_2$，得到

$$
\boxed{d_m^{\mathrm{cert}}
\le\|\mathbf p-\mathbf c_m\|_2-r_{\mathrm{uav}}-r_m.}
\tag{11}
$$

这一步保证距离正则化本身不会高估几何间隙。

### 3.3 严格保守的加权 LogSumExp

任意正概率 $0<P_m<1$ 直接作为未归一化 LogSumExp 权重，**不能**保证软距离小于最小距离，因为 $-\sigma\ln P_m>0$。

为继续利用占据分数，又不让低占据概率削弱认证，定义**不小于 1 的证书权重**

$$
\boxed{\kappa_m=1+\alpha P_m\ge1,\qquad\alpha\ge0,}
\tag{12}
$$

并定义

$$
\boxed{
\begin{aligned}
h_{\mathrm{cert}}(\mathbf p,t)
={}&-\sigma\ln\left(
\sum_{m\in\mathcal I(t)}
\kappa_m\exp\left(-\frac{d_m^{\mathrm{cert}}(\mathbf p,t)}{\sigma}\right)
\right)\\
&-d_{\mathrm{margin}},\qquad \sigma>0.
\end{aligned}}
\tag{13}
$$

其中 $d_{\mathrm{margin}}>0$ 是期望的实体间安全裕度。若不希望引入概率权重，则令 $\alpha=0$，此时 $\kappa_m\equiv1$，是最简单的无权严格保守方案。

**可选的原方案最小改动**：如果所有认证体素的占据值均满足经过验证的统一下界 $P_m\ge P_{\mathrm{ref}}>0$，亦可用 $\kappa_m=P_m/P_{\mathrm{ref}}\ge1$。但这并不授权排除低概率且仍可能存在真实障碍物的体素；几何覆盖 (9) 依然是独立前提。

记 $d_{\min}^{\mathrm{cert}}=\min_{m\in\mathcal I}d_m^{\mathrm{cert}}$，取达到该最小值的索引 $j$，则

$$
\sum_{m\in\mathcal I}\kappa_m e^{-d_m^{\mathrm{cert}}/\sigma}
\ge\kappa_j e^{-d_j^{\mathrm{cert}}/\sigma}
\ge e^{-d_{\min}^{\mathrm{cert}}/\sigma}.
$$

两边施加 $-\sigma\ln(\cdot)$，得到

$$
\boxed{h_{\mathrm{cert}}(\mathbf p,t)
\le d_{\min}^{\mathrm{cert}}(\mathbf p,t)-d_{\mathrm{margin}}.}
\tag{14}
$$

定义真实障碍物与球形 UAV 外包络的最小有符号距离

$$
D_{\mathrm{phys}}(\mathbf p,t)
=\inf_{\mathbf q\in\mathcal O_{\mathrm{real}}(t)}
\|\mathbf p-\mathbf q\|_2-r_{\mathrm{uav}}.
$$

对任意真实障碍物点 $\mathbf q$，由 A2 存在体素球 $m$ 使 $\|\mathbf q-\mathbf c_m\|_2\le r_m$。三角不等式与 (11) 给出

$$
\min_jd_j^{\mathrm{cert}}
\le d_m^{\mathrm{cert}}
\le\|\mathbf p-\mathbf q\|_2-r_{\mathrm{uav}}.
$$

对真实障碍物点取下确界，得到 $d_{\min}^{\mathrm{cert}}\le D_{\mathrm{phys}}$。结合 (14)：

$$
\boxed{h_{\mathrm{cert}}\ge0
\ \Longrightarrow\
D_{\mathrm{phys}}\ge d_{\mathrm{margin}}>0.}
\tag{15}
$$

这才完成**从认证场非负到真实实体有安全间隙**的严格桥接。结论依赖 A2；DSP 的概率占据本身不能替代 A2。

---

## 4. 随时间变化的 3D Field-HOCBF 精确微分

在**单个固定认证区间**内，假设体素索引集 $\mathcal I$、半径 $r_m$、权重 $\kappa_m$ 均固定，中心轨迹 $\mathbf c_m(t)$ 至少二次连续可微；使用动基座系一致的体素速度与加速度

$$
\mathbf v_m=\dot{\mathbf c}_m,\qquad
\mathbf a_m=\ddot{\mathbf c}_m.
$$

定义

$$
\mathbf n_m=\frac{\mathbf p-\mathbf c_m}{s_m},\qquad
\mathbf v_{\mathrm{rel},m}=\mathbf v-\mathbf v_m.
\tag{16}
$$

**注意**：$\mathbf n_m$ 是正则化距离梯度，范数不大于 1，不能未经说明称为严格的单位向量。

定义 LogSumExp 的内部归一化权重

$$
\boxed{w_m=\frac{\kappa_m e^{-d_m^{\mathrm{cert}}/\sigma}}
{\sum_j\kappa_j e^{-d_j^{\mathrm{cert}}/\sigma}},\qquad
\sum_m w_m=1.}
\tag{17}
$$

这仅是**对数求导产生的 Softmax 权重**，不同于将原距离场中的占据加权和除以 $\sum P_m$ 的错误归一化。

### 4.1 一阶导数

$$
\boxed{
\mathbf g=\nabla_{\mathbf p}h_{\mathrm{cert}}=\sum_mw_m\mathbf n_m,
\qquad
\dot h_{\mathrm{cert}}=\sum_mw_m\mathbf n_m^\top\mathbf v_{\mathrm{rel},m}.}
\tag{18}
$$

由于体素中心随时间运动，严格地说安全函数是 $h_{\mathrm{cert}}(\mathbf p,t)$，不是只依赖 $\mathbf p$ 的自治函数。

### 4.2 二阶导数

对每一项：

$$
\dot d_m^{\mathrm{cert}}=\mathbf n_m^\top\mathbf v_{\mathrm{rel},m}.
$$

$$
\boxed{
\ddot d_m^{\mathrm{cert}}
=\frac{\|\mathbf v_{\mathrm{rel},m}\|_2^2-
(\mathbf n_m^\top\mathbf v_{\mathrm{rel},m})^2}{s_m}
+\mathbf n_m^\top(\dot{\mathbf v}-\mathbf a_m).}
\tag{19}
$$

固定 $\kappa_m$ 时：

$$
\boxed{
\ddot h_{\mathrm{cert}}=
\sum_mw_m\ddot d_m^{\mathrm{cert}}
-\frac1\sigma\left[
\sum_mw_m(\dot d_m^{\mathrm{cert}})^2
-\left(\sum_mw_m\dot d_m^{\mathrm{cert}}\right)^2
\right].}
\tag{20}
$$

代入式 (1) 中的名义动力学，得

$$
\boxed{
\ddot h_{\mathrm{nom}}
=\mathbf A^\top\mathbf u+\ell_{\mathrm{nom}},
\qquad\mathbf A=\boldsymbol\Lambda^\top\mathbf g.}
\tag{21}
$$

其中 $\ell_{\mathrm{nom}}$ 是把 (19)—(20) 中 $\dot{\mathbf v}$ 换成 $-\boldsymbol\Lambda\mathbf v+\hat{\mathbf a}_{\mathrm{ni}}$ 后得到的所有无输入项，**包括**时变障碍物中心的速度、加速度及 Softmax 速度方差修正。

真实二阶导数则满足

$$
\boxed{\ddot h_{\mathrm{real}}
=\mathbf A^\top\mathbf u+\ell_{\mathrm{nom}}
+\mathbf g^\top(\Delta\mathbf a_{\mathrm{ni}}+\mathbf d_v).}
\tag{22}
$$

仅当使用了**真实相关状态**下的导数，或已认证地覆盖了使用冻结预测状态造成的偏差时，式 (22) 才能直接用于在线控制实现。

**切换说明**：一旦点集、$\kappa_m$、体素位置或运动模型发生不连续变化，必须采用分段证明，并重新验证切换后 $h\ge0$、$\psi_1\ge0$；若还希望描述跳变造成的测量不确定性，则需增加切换安全证书。不能在原来的 $C^2$ 导数公式中直接跨越不连续时刻。

---

## 5. Hard Robust HOCBF 与前向不变性

设 $\gamma_1,\gamma_2>0$，定义

$$
\psi_0=h_{\mathrm{cert}},\qquad
\psi_1=\dot h_{\mathrm{cert}}+\gamma_1h_{\mathrm{cert}},\qquad
\psi_2=\dot\psi_1+\gamma_2\psi_1.
\tag{23}
$$

由 (21) 定义名义右端

$$
\boxed{b_{\mathrm{nom}}=-\left[\ell_{\mathrm{nom}}+
(\gamma_1+\gamma_2)\dot h_{\mathrm{cert}}
+\gamma_1\gamma_2h_{\mathrm{cert}}\right].}
\tag{24}
$$

由 (22) 可得

$$
\boxed{
\psi_{2,\mathrm{real}}
=\mathbf A^\top\mathbf u-b_{\mathrm{nom}}
+\mathbf g^\top(\Delta\mathbf a_{\mathrm{ni}}+\mathbf d_v).}
\tag{25}
$$

采用无松弛的硬鲁棒安全约束

$$
\boxed{\mathbf A^\top\mathbf u\ge b_{\mathrm{nom}}+\rho_{\mathrm{dyn}}.}
\tag{26}
$$

### 定理 1：单个认证区间的条件性鲁棒前向不变性

**假设**：A1、A2 成立；认证障碍物场满足第 4 节的分段 $C^2$ 条件；初态满足

$$
\boxed{h_{\mathrm{cert}}(0)\ge0,\qquad\psi_1(0)\ge0;}
\tag{27}
$$

闭环解存在且具有所需正则性；并且**在整个连续时间区间内实际施加的输入**满足真实执行器限制与 (26)。

**结论**：对所有允许的非惯性动力学扰动，

$$
\boxed{h_{\mathrm{cert}}(t)\ge0,
\quad D_{\mathrm{phys}}(t)\ge d_{\mathrm{margin}}>0.}
\tag{28}
$$

**证明**：由 (7)、(25)、(26)，对所有允许扰动有

$$
\psi_{2,\mathrm{real}}
\ge(b_{\mathrm{nom}}+\rho_{\mathrm{dyn}})
-b_{\mathrm{nom}}-\rho_{\mathrm{dyn}}=0.
$$

于是 $\dot\psi_1+\gamma_2\psi_1\ge0$。乘积分因子并积分，得

$$
\psi_1(t)\ge\psi_1(0)e^{-\gamma_2t}\ge0.
$$

再由 $\dot h_{\mathrm{cert}}+\gamma_1h_{\mathrm{cert}}=\psi_1\ge0$，得

$$
\boxed{h_{\mathrm{cert}}(t)
\ge h_{\mathrm{cert}}(0)e^{-\gamma_1t}\ge0.}
$$

最后由 (15)，得 $D_{\mathrm{phys}}(t)\ge d_{\mathrm{margin}}$。证毕。

> **术语准确性**：该定理证明从满足 $h\ge0$ 和 $\psi_1\ge0$ 的初始状态出发，闭环轨迹保持在指定认证安全条件中；不能说整个只由 $h\ge0$ 定义的位置集合对**任意初始速度**都前向不变。

### 推论：多个认证区间的预测更新安全衔接

设更新时刻 $0=t_0<t_1<t_2<\cdots$，在任意有限区间内更新次数有限。每个区间采用经认证的预测误差界与硬约束 (26)，且在切换时刻真实状态满足新场的 $h(t_j^+)\ge0$、$\psi_1(t_j^+)\ge0$，并且新控制约束仍能持续执行，则定理 1 可在各区间反复应用。

如果更新**仅改变名义动力学与有效扰动界、并不改变实际安全函数**，则 $h$ 不会因预测更新本身跳变；在相应可微性条件下可直接衔接比较证明。若障碍物覆盖或安全函数随更新变化，必须单独验证切换条件。

> **边界**：这一推论仍把“新旧 Hard Robust HOCBF 约束能够持续执行”作为前提。只把 Slack 设为零、或确认预测误差不超过阈值，**都不足以证明递推可行性**。

---

## 6. 从定理到 `field_hocbf.h` 的最小实现对应

下面列出**审查要点**，不把未实现的模块描述成现有代码功能。

1. **距离和 3D 梯度**：不得再清零位置差的 Z 分量。每个体素使用 $s_m=\sqrt{\|\mathbf p-\mathbf c_m\|^2+\epsilon^2}$ 计算 $\mathbf n_m=(\mathbf p-\mathbf c_m)/s_m$；同时使用 $d_m^{\mathrm{cert}}=s_m-\epsilon-r_{\mathrm{uav}}-r_m$ 进入 LogSumExp。
2. **二阶距离导数的分母**：必须使用 $s_m$，不能把可能为负的认证间隙 $d_m^{\mathrm{cert}}$ 放进曲率项分母。
3. **证书权重与数值稳定**：可使用 $\kappa_m=1+\alpha P_m$（固定于当前认证区间），以及以最小间隙为锚的移位 LogSumExp：

   $$
   -\sigma\ln\sum_m\kappa_m e^{-d_m^{\mathrm{cert}}/\sigma}
   =d_{\min}^{\mathrm{cert}}
   -\sigma\ln\sum_m\kappa_m
   e^{-(d_m^{\mathrm{cert}}-d_{\min}^{\mathrm{cert}})/\sigma}.
   $$

   不应重新除以 $\sum_m P_m$；导数仍采用 $w_m=\kappa_m e^{-d_m/\sigma}/\sum_j\kappa_j e^{-d_j/\sigma}$。
4. **扰动修正**：在名义右端 $b_{\mathrm{nom}}$ 上增加式 (6) 或更紧的已证明有效上界；不得只加入一阶角速度误差而省略二阶项。
5. **输入矩阵**：一般表达式为 $\mathbf A=\boldsymbol\Lambda^\top\nabla h$；由于当前模型的 $\boldsymbol\Lambda$ 为对角阵，与已有 `lambda.cwiseProduct(grad_h)` 一致。
6. **Hard 约束必须真实有效**：实际 ACADO 模型还有 `delta0` 等松弛变量、`field_active` 开关、状态反馈修正项以及其他障碍物/输入硬约束。应检查最终构造的优化行与 (26) 是否等价，不能只检查 `result.b`。
7. **认证点集覆盖**：`prune()`、占据阈值、垂直筛选或视距筛选可能导致 A2 失效；若没有证书，不能在安全证明中默认为障碍物已经被全部覆盖。空点集也不能在严格认证场中自动被视为自由空间。
8. **采样与实物闭环**：10 Hz ACADO 仅在离散节点施加约束不能直接推出两节点之间的安全。必须另外建立采样间裕度、可达管束安全认证或其他有效的采样控制证明。

## 7. 尚未被本定理解决的问题

- **递推可行性**：需要证明硬约束对真实速度/加速度边界及全部激活障碍物联合可行，而且执行安全控制后仍保有后续可行输入；这是下一篇定理或后续章节的任务。
- **因果误差包线的可验证性**：应包括预测时域增长、UGV 机动许可与通信静默期；统计样本误差不能直接替代最坏情况包线。
- **事件触发切换**：需保证在旧包线失效前完成更新，满足新认证场的切换条件，且更新期内安全控制仍可执行。
- **动态感知覆盖**：概率 DSP 输出本身不构成对所有真实障碍物的确定性覆盖证书；若没有该证书，必须如实报告“相对于已认证/已感知障碍物”的安全范围。

### 结论

本修订版保留了原证明正确的主线：**非惯性有界扰动 $\rightarrow$ 鲁棒 HOCBF 收缩 $\rightarrow$ 二阶比较引理 $\rightarrow$ 认证安全场前向不变 $\rightarrow$ 物理间隙保证**。新增的几何覆盖条件与保守光滑距离使最后一步有严格的条件性数学依据；但系统的连续时间输入可实施性、离散采样安全以及递推可行性仍需独立证明。