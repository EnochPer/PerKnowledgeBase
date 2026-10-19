---
title: ICP 中只优化旋转的 RPY 与 SO(3) 李代数实现
date: 2026-09-22
tags: [ICP, Ceres, SO(3), 李代数, RPY, 位姿优化]
---
# ICP 中只优化旋转的 RPY 与 SO(3) 李代数实现

## 1. 问题边界

设 source 点云为 $p_i$，target 点云为 $q_j$，变换方向约定为 source 到 target：

$$
q_i \simeq R p_i+t,\qquad R\in SO(3),\quad t\in\mathbb{R}^3 .
$$

这里的 `xyz` 是齐次矩阵

$$
T=\begin{bmatrix}R&t\\0^\top&1\end{bmatrix}
$$

中的平移列 $t$。因此固定 $t$ 的几何含义是：绕 source 坐标系原点旋转，再加上一个固定平移。

注意：若实际机构绕非原点 $c$ 旋转，应先把模型写成

$$
q=R(p-c)+c+t'
$$

否则会把旋转中心和 `xyz` 的含义混在一起。

本文的“只优化旋转”指：每次 Ceres 求解时参数块只有旋转增量，$t=t_0$ 不进入参数块。平移约束或置信度只用于以下两种事情：

1. 从外部候选平移中筛选 $t_0$，再固定它求旋转；
2. 把平移也放进参数块，并加入先验或约束。后一种已经是 $(R+t)$ 联合优化，不再是纯旋转问题。

如果平移候选为 $t^{(k)}$，给定初值 $t_0$ 的高斯置信度可以写成

$$
w_t(t^{(k)})=\exp\left[-\frac12(t^{(k)}-t_0)^\top\Sigma_t^{-1}(t^{(k)}-t_0)\right].
$$

工程上可以对每个候选 $t^{(k)}$ 单独运行旋转 ICP，再比较

$$
J_k=\sum_i\rho\left(\left\|R_kp_i+t^{(k)}-q_{\pi_k(i)}\right\|^2\right)-\log w_t(t^{(k)}).
$$

若希望 $t$ 连续地参与求解，则应使用平移先验

$$
r_t=L(t-t_0),\qquad L^\top L=\Sigma_t^{-1},
$$

这需要增加 `t[3]` 参数块，文末会说明这种扩展的边界。

## 2. RPY 到旋转矩阵

采用 Z-Y-X 约定，`roll`、`pitch`、`yaw` 分别为 $\phi$、$\theta$、$\psi$，单位统一使用弧度：

$$
R_{\mathrm{init}}=R_z(\psi)R_y(\theta)R_x(\phi).
$$

展开为

$$
R=\begin{bmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi-s_\psi c_\phi & c_\psi s_\theta c_\phi+s_\psi s_\phi\\
s_\psi c_\theta & s_\psi s_\theta s_\phi+c_\psi c_\phi & s_\psi s_\theta c_\phi-c_\psi s_\phi\\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix},
$$

其中 $c_x=\cos(x)$，$s_x=\sin(x)$。RPY 适合作为配置、日志和最终输出，但不建议直接作为优化变量：当 `pitch` 接近 $\pm90^\circ$ 时出现万向锁，`roll` 与 `yaw` 的 Jacobian 变得相关，数值步长会异常。

不要把欧拉角向量简单地当成李代数向量。**正确做法是先由 RPY 得到 $R_\mathrm{init}$，再在 $R_\mathrm{init}$ 附近优化局部增量。**

## 3. SO(3) 局部李代数参数化

对三维向量 $\delta\phi=[\delta_x,\delta_y,\delta_z]^\top$ 定义反对称矩阵

$$
[\delta\phi]_\times=
\begin{bmatrix}
0&-\delta_z&\delta_y\\
\delta_z&0&-\delta_x\\
-\delta_y&\delta_x&0
\end{bmatrix}.
$$

本文使用左扰动（增量在 target/world 坐标系表达）：

$$
R(\delta\phi)=\operatorname{Exp}([\delta\phi]_\times)R_\mathrm{init}.
$$

当 $\|\delta\phi\|=\alpha$ 时，指数映射等价于角轴旋转

$$
\operatorname{Exp}([\delta\phi]_\times)=I+\frac{\sin\alpha}{\alpha}[\delta\phi]_\times+
\frac{1-\cos\alpha}{\alpha^2}[\delta\phi]_\times^2 .
$$

代码中直接用 `ceres::AngleAxisToRotationMatrix` 计算该指数映射，避免手写 `Jet` 类型的三角函数分支。每次外层 ICP 迭代都令 `delta = {0,0,0}`，求解后更新

$$
R_\mathrm{init}\leftarrow\operatorname{Exp}([\delta\phi^*]_\times)R_\mathrm{init},
$$

然后重新建立对应关系。这样优化变量始终是小量，远离全局 RPY 的奇异点。若使用右扰动，则应始终写成 $R=R_\mathrm{init}\operatorname{Exp}([\delta\phi]_\times)$，不能混用两种 Jacobian。

左扰动下，点到点残差对增量的近似 Jacobian 为

$$
\frac{\partial(Rp)}{\partial\delta\phi}\bigg|_{0}=-[R_\mathrm{init}p]_\times .
$$

点到平面残差 $n_i^\top(Rp_i+t-q_i)$ 的 Jacobian 为

$$
J_i=-n_i^\top[R_\mathrm{init}p_i]_\times .
$$

**若姿态可能跨过大角度或接近 $180^\circ$，应采用多次小增量更新，或改用带流形的四元数参数化。**

## 4. 固定平移时的 ICP 目标

对应关系 $\pi(i)$ 固定后，点到点目标为

$$
\min_{\delta\phi}\ \sum_i\rho\left(\left\|W_i^{1/2}\left(\operatorname{Exp}([\delta\phi]_\times)R_\mathrm{init}p_i+t_0-q_{\pi(i)}\right)\right\|^2\right)
+\lambda_R\|\delta\phi\|^2 .
$$

其中 $W_i$ 是测量权重，$\rho$ 可以取 Huber 或 Cauchy 鲁棒核，最后一项是“不要离当前姿态太远”的软先验。若当前增量就是局部坐标中的旋转误差，则先验残差可直接写成

$$
r_R=\sqrt{\lambda_R}\,\delta\phi .
$$

点到平面版本只需把三维残差替换为

$$
r_i=n_i^\top\left(\operatorname{Exp}([\delta\phi]_\times)R_\mathrm{init}p_i+t_0-q_{\pi(i)}\right).
$$

点到平面通常收敛更快，但需要可靠且单位化的 target 法向量；平面内滑动方向不会被该残差观测到。

固定 $t$ 时，不能把 `t` 放入 Ceres 后再依赖某个未设置的默认值。最清晰的实现是把 `t0` 作为 cost functor 的常量成员，参数块只有 `delta[3]`。同理，若 `t` 的距离置信度只用于筛选候选，就不能把它误写成旋转残差的权重。

## 5. ICP 外层流程

```text
输入：source、target、初始 RPY、固定平移 t0
由 RPY 得到 R_init
重复最多 max_outer 次：
  1. 用 R_init p_i + t0 变换 source
  2. 在 target 中找最近邻，按距离、法向夹角和互为最近邻剔除坏对应
  3. 对每个对应建立 AutoDiffCostFunction<..., 3, 3>
  4. 用 delta = 0、Huber/Cauchy 和可选旋转先验运行 Ceres
  5. R_init <- Exp(delta^) R_init，delta 清零
  6. 检查旋转增量、RMS 相对下降、有效对应数和 Hessian 条件数
直到收敛或对应点不足
输出 R_init，并按 ZYX 顺序转回 roll、pitch、yaw
```

最近邻搜索在示例中使用暴力方法，复杂度为 $O(NM)$，生产环境应替换为 KD-tree。对应关系在每次外层迭代中冻结；不要在 Ceres 的一次内层求解过程中改变对应关系，否则残差函数不再连续。

推荐同时检查以下条件：

- 有效对应数高于最小阈值，且空间分布不能集中在一个小区域；
- 旋转增量 $\|\delta\phi\|$ 小于阈值，例如 $10^{-4}$ rad；
- RMS 的相对下降小于阈值；
- $J^\top J$ 的最小特征值和条件数正常。

固定平移的误差不会被优化消除，而会被旋转部分代偿。若 `t0` 有系统偏差，常见表现是 RMS 有不可下降的下限，或更换点云子集后估计出的旋转明显变化。此时应重新评估平移先验，而不是只调 Huber 阈值。

## 6. Ceres 实现要点

### 6.1 残差块

点到点 functor 的核心形式如下，完整可运行版本见同目录的 [`demo.cpp`](./demo.cpp)：

```cpp
struct PointToPointResidual {
  Eigen::Vector3d p, q, t_fixed;
  Eigen::Matrix3d R0;

  template <typename T>
  bool operator()(const T* delta, T* residual) const {
    T aa[3] = {delta[0], delta[1], delta[2]};
    T Rd[9];
    ceres::AngleAxisToRotationMatrix(aa, ceres::RowMajorAdapter3x3(Rd));
    T p0[3];
    for (int r = 0; r < 3; ++r) {
      p0[r] = T(R0(r, 0)) * T(p.x()) + T(R0(r, 1)) * T(p.y()) +
              T(R0(r, 2)) * T(p.z());
    }
    for (int r = 0; r < 3; ++r) {
      residual[r] = Rd[3 * r + 0] * p0[0] +
                    Rd[3 * r + 1] * p0[1] +
                    Rd[3 * r + 2] * p0[2] + T(t_fixed[r]) - T(q[r]);
    }
    return true;
  }
};
```

添加残差时使用 `AutoDiffCostFunction<PointToPointResidual, 3, 3>`。`delta` 是三维角轴/李代数向量，不是 RPY；单位是弧度。`t_fixed`、`R0` 和点坐标是常量，必须显式转换为 `T`，否则会在 `ceres::Jet` 实例化时产生类型混用。

### 6.2 鲁棒核、权重和先验

若点的位置标准差为 $\sigma_i$，先把残差乘以 $1/\sigma_i$，再使用 Huber 核。Huber 的阈值也应以米为单位，不能直接使用弧度阈值。旋转先验的 $\lambda_R$ 以 $1/\mathrm{rad}^2$ 标定；过大时会压制真实旋转，过小时对退化几乎没有帮助。

若需要在联合优化中使用平移置信度，可增加：

```cpp
struct TranslationPrior {
  Eigen::Vector3d t0;
  double inv_sigma;
  template <typename T>
  bool operator()(const T* t, T* r) const {
    for (int k = 0; k < 3; ++k)
      r[k] = T(inv_sigma) * (t[k] - T(t0[k]));
    return true;
  }
};
```

但这时参数块是 `delta[3]` 和 `t[3]`，目标变成带软先验的六自由度 ICP。软先验允许越界；若必须硬性限制每个轴的范围，可使用 `SetParameterLowerBound/UpperBound`。Ceres 原生 bound 是轴对齐盒约束，不能直接表示

$$
\|t-t_0\|\le r
$$

这样的球约束。球约束需要重参数化 $t=t_0+r\,s/\sqrt{1+\|s\|^2}$、自定义流形或外层投影，并且这些方案都已经超出“只优化旋转”的参数化。

### 6.3 版本与数值细节

- Ceres 的 `EulerAnglesToRotationMatrix` 存在单位和分量命名差异，不应直接拿来实现工程 RPY；示例自行用 Eigen 构造 ZYX 旋转。
- 弧度、米和残差标准差要分开管理，禁止把角度和距离直接相加比较。
- 点到平面法向量先归一化，法向方向必须与 target 坐标系一致。
- RPY 输出时将 `asin` 输入截断到 $[-1,1]$，并处理 `pitch` 接近 $\pm90^\circ$ 的退化分支；旋转误差用 $SO(3)$ 测地角，不用三个 RPY 分量差。
- 180 度附近角轴不唯一；若初值误差很大，应使用多层 ICP、四元数流形或先做粗配准。
- 空对应集、NaN 坐标、法向退化和异常大的最近邻距离必须显式报错或退出，不能用默认值静默掩盖。

## 7. 验收与调参清单

1. 用合成数据检查 `RPY -> R -> RPY` 往返误差，并确认变换方向是 $q=Rp+t$。
2. 令 `delta=0` 时残差应对应输入 RPY 和固定 `t0`，每次更新后重新建立对应。
3. 用无噪声、已知对应关系检查最终旋转测地角；有噪声时同时报告对应数、RMS、增量和条件数。
4. 改变点云子集后若旋转显著变化，优先检查几何退化和 `t0` 偏差。
5. `d_max` 可从粗到细收紧；Huber 阈值取内点噪声的约 $1\sim2$ 倍。
6. 生产环境替换最近邻实现后，保持对应筛选和内层残差定义不变。

运行示例（仅示意，本文未执行编译）：

```bash
cd 03-Knowledges/标定/带约束的位姿优化
g++ -std=c++14 demo.cpp -lceres -lglog -I/usr/include/eigen3 -o demo
./demo
```

实际项目应使用系统的 CMake/Ceres 配置，并根据安装位置补充链接参数。按照仓库规则，本文任务未执行编译。

## 8. 参考

- Ceres Solver：[`AutoDiffCostFunction`](http://ceres-solver.org/nnls_modeling.html)、[`LossFunction`](http://ceres-solver.org/nnls_modeling.html)、[`Covariance`](http://ceres-solver.org/nnls_covariance.html)。
- Besl 与 McKay，*A Method for Registration of 3-D Shapes*，经典 ICP。
- Barfoot，*State Estimation for Robotics*，$SO(3)$ 指数映射与局部扰动。


## 关联实验：仅优化旋转的 GICP

[仅优化旋转的 GICP：给定并固定平移](../仅优化旋转的gicp/README.md) 将固定平移的 SO(3) 增量思路迁移到 small_gicp，采用右扰动和三维旋转块 LM 求解，记录旋转 Hessian 特征值，并提供正确/有偏平移两组独立实验。代码尚未编译运行，结果需另行验证。
