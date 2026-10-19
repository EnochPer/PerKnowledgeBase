# small_gicp 最小 C++ Demo

[small_gicp](https://github.com/koide3/small_gicp) 是用于点云精配准的 C++/Python 库，提供 ICP、Point-to-Plane ICP、GICP 和 VGICP 等算法。C++ 核心是头文件库，要求 C++17；Eigen 是必需依赖，nanoflann 和 Sophus 随仓库提供。OpenMP/TBB 用于并行预处理和计算，PCL 只在使用 PCL 适配接口时需要。

## 本机检查结果

检查时间：2026-10-15。

| 项目 | 结果 |
| --- | --- |
| 系统 | Ubuntu 22.04 (x86_64) |
| C++ 编译器 | GCC 11.4，满足 C++17 |
| CMake | 3.22.1 |
| Eigen | 3.4.0，已安装 `/usr/include/eigen3` |
| OpenMP | GCC OpenMP 运行库可见；`libomp-dev` 头文件尚未单独确认 |
| Intel TBB | 运行库可见，可选 |
| PCL | 未发现 `/usr/include/pcl`；本 demo 不使用 PCL |
| small_gicp | 本知识库中尚未安装源码，需要单独获取源码目录 |

因此，当前系统具备运行无 PCL C++ demo 的基础工具和 Eigen；还缺少 small_gicp 源码本身。没有安装依赖或执行编译。

## Demo 内容

`demo.cpp` 生成三个互相垂直平面组成的合成点云，施加一个已知旋转和平移得到 target，然后调用 `small_gicp::align` 从单位初值估计 `T_target_source`。程序会输出收敛状态、迭代次数、内点数量、估计变换以及和真值的平移/旋转误差。

## 获取源码并编译

下面命令会从网络获取源码，并会触发 CMake 配置和编译；执行前请确认依赖和编译授权。

```bash
git clone https://github.com/koide3/small_gicp.git /path/to/small_gicp
cmake -S 03-Knowledges/标定/small_gicp \
  -B /tmp/small_gicp_demo-build \
  -DSMALL_GICP_DIR=/path/to/small_gicp \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/small_gicp_demo-build -j2
/tmp/small_gicp_demo-build/small_gicp_demo
```

如果源码放在 demo 目录下的 `small_gicp/`，可以省略 `-DSMALL_GICP_DIR`。PCL 不需要安装。若 CMake 报 OpenMP 找不到，先使用单线程配置也能保留算法功能；若仓库版本要求 OpenMP 开发包，再按报错安装对应系统包。

预期程序返回码为 0，并显示 `converged: true`；误差阈值在 demo 中设为平移小于 1 cm、旋转小于 0.01 rad。


## GICP 逐次迭代的 Hessian 分析

以下为用户提供的实际运行日志及其分析，本次文档整理未重新编译或运行。上文环境检查是初始状态；后续源码已放在 `/home/zzh/small_gicp-master/small_gicp`，用户已成功构建并运行 demo。当前 demo 使用 `Registration<GICPFactor, ParallelReductionOMP, HessianLogger>` 接口，在每次线性化时打印 Hessian；预处理仍通过 helper 库完成。CMake 已将两个 helper 实现源文件构建为静态库并链接，不能只添加头文件路径。

### 实验设置与矩阵含义

场景为三个互相垂直平面组成的无噪声合成点云，初始位姿为单位矩阵。真值平移为 `(0.12, -0.08, 0.06)` 米，旋转为 $R_z(0.08)R_y(-0.05)$，角度单位为弧度。使用 2 个线程、0.05 米降采样分辨率、0.5 米最大对应距离及 10 个邻点估计协方差。

GICP 在当前对应关系和权重下构造 Gauss–Newton 近似矩阵：

$$H=\sum_k J_k^\top W_kJ_k=V\Lambda V^\top$$

日志中的特征值来自未加 LM 阻尼的矩阵，而不是求解时使用的 $H+\mu I$。这里用 $\mu$ 表示日志中的 `lambda`，以区别于 Hessian 的特征值 $\lambda_i$。

最弱特征向量的分量顺序为 `[rx ry rz tx ty tz]`，对应右乘位姿增量。它描述旋转和平移的组合方向，不能将第几个特征值直接解释成第几个自由度。

### 原始运行日志

```text
--- LM optimization ---
[Hessian] iter=0 e=9.558882e+02
  eigenvalues (ascending): 4.713707e+04 5.126888e+04 6.335915e+04 6.355599e+04 6.755745e+04 1.010682e+05
  raw lambda_min/lambda_max: 4.663889e-01
  weakest direction [rx ry rz tx ty tz]:  1.623622e-02  1.600773e-02 -9.037672e-01 -4.257363e-01 -3.645711e-02 -1.022452e-02
iter=0 inner=0 e=9.558882e+02 new_e=1.085054e+01 lambda=1.000000e-03 dt=1.549543e-01 dr=9.311886e-02
[Hessian] iter=1 e=2.848141e-01
  eigenvalues (ascending): 1.462298e+05 1.465382e+05 1.472102e+05 1.918852e+05 1.924642e+05 1.954748e+05
  raw lambda_min/lambda_max: 7.480749e-01
  weakest direction [rx ry rz tx ty tz]:  2.307559e-01 -8.755548e-01  4.238486e-01 -2.138890e-02  3.834544e-03 -5.971598e-03
iter=1 inner=0 e=2.848141e-01 new_e=4.645177e-08 lambda=1.000000e-04 dt=1.332142e-03 dr=1.241379e-03
[Hessian] iter=2 e=4.646357e-08
  eigenvalues (ascending): 1.462654e+05 1.465840e+05 1.472396e+05 1.919345e+05 1.925105e+05 1.955257e+05
  raw lambda_min/lambda_max: 7.480620e-01
  weakest direction [rx ry rz tx ty tz]:  2.305818e-01 -8.819385e-01  4.104584e-01 -2.217179e-02  3.664654e-03 -5.952936e-03
iter=2 inner=0 e=4.646357e-08 new_e=2.558364e-28 lambda=1.000000e-05 dt=6.912797e-07 dr=4.352875e-09
converged: true
iterations: 2
inliers: 1261
estimated T_target_source:
 0.995556 -0.079915 -0.049819  0.120000
 0.079815  0.996802 -0.003994 -0.080000
 0.049979  0.000000  0.998750  0.060000
 0.000000  0.000000  0.000000  1.000000
translation error [m]: 0.000000
rotation error [rad]: 0.000000
```

### 特征值与退化判断

定义原始尺度下的比值及条件数：

$$\rho=\frac{\lambda_{\min}}{\lambda_{\max}},\qquad \kappa_2(H)=\frac{\lambda_{\max}}{\lambda_{\min}}$$

此处条件数公式适用于正定的对称矩阵，本次打印的特征值均为正。

| 外层迭代编号 | 最小特征值 | 最大特征值 | 比值 $\rho$ | 条件数约值 |
| --- | --- | --- | --- | --- |
| 0 | 47137.07 | 101068.2 | 0.4663889 | 2.144 |
| 1 | 146229.8 | 195474.8 | 0.7480749 | 1.337 |
| 2 | 146265.4 | 195525.7 | 0.7480620 | 1.337 |

三个比值均远高于示例阈值 $10^{-3}$～$10^{-4}$，在本次坐标、尺度和对应关系下，没有出现明显的相对弱约束退化。第二次线性化后谱分布趋于稳定，与三个平面提供多个方向约束的场景相符。

`weakest direction` 只是六个方向中相对最弱的方向，本次并不代表退化方向。初始最弱向量的主要分量为 `rz` 和 `tx`；后两次主要是旋转分量。但旋转和平移量纲不同，这种分量比较仅针对当前坐标参数化。最小的几个特征值很接近时，特征向量还可能对微小扰动敏感；向量整体符号也没有物理区别。

### LM 收敛过程

- 实际进行了 **3 次外层迭代**，编号为 0、1、2。当前库实现将最后一次零起始编号赋给 `result.iterations`，所以打印的是 `iterations: 2`，不能解读为只执行了两次。
- 三次都在 `inner=0` 接受更新，没有额外阻尼重试。阻尼从 $10^{-3}$ 依次降低到 $10^{-4}$、$10^{-5}$。
- `dt` 是平移增量范数（米），`dr` 是旋转增量范数（弧度）。最后一次分别为 $6.912797\times10^{-7}$ 和 $4.352875\times10^{-9}$，已低于当前停止阈值。
- 上一步 `new_e` 不必等于下一步 `e`：内层评估沿用本轮对应关系和协方差权重，下一次外层线性化会重新搜索对应点并更新权重。例如 `10.85054` 到 `0.2848141` 的变化不能单独视为一次额外位姿更新的效果。
- 最终内点数为 1261；估计平移与真值一致，平移及旋转误差在六位小数输出精度下为零，不表示浮点误差严格为零。

### 结论与适用范围

此次日志证明 demo 在本机完成了该合成场景的 GICP 配准，并且沿途 Hessian 的原始尺度条件数较小。`converged: true` 与特征值分析分别反映停止条件和局部约束强度，二者不能互相替代。

本次未做旋转/平移尺度归一化，也没有验证噪声、部分重叠、单平面、走廊或较差初值。不能直接将这里的特征值大小或阈值推广到其他场景。真实数据上还应结合有效对应点数、残差及位姿合理性判断；整个 Hessian 接近零时，仅看比值不足以判断约束是否充足。


## 关联实验：仅优化旋转的 GICP

[仅优化旋转的 GICP：给定并固定平移](../仅优化旋转的gicp/README.md) 将固定平移的 SO(3) 增量思路迁移到 small_gicp，采用右扰动和三维旋转块 LM 求解，记录旋转 Hessian 特征值，并提供正确/有偏平移两组独立实验。代码尚未编译运行，结果需另行验证。


## 接口、缓存与标定精度学习

[small_gicp 示例学习：接口、缓存复用与标定精度](./small_gicp示例学习：接口、缓存复用与标定精度.md) 对照 01/02/03 示例，解释 PCL 接口取舍、连续帧缓存交换、GICP/VGICP 完整流程，以及好初值下的标定精度与验证方法。
