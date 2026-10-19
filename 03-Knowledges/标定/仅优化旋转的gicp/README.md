# 仅优化旋转的 GICP：给定并固定平移

## 与已有知识的关系

- [带约束的位姿优化：只优化旋转 RPY 与 SO(3)](../带约束的位姿优化/只优化旋转rpy.md)：问题定义、旋转中心、Ceres 左扰动实现和带偏平移的影响。
- [small_gicp 基础实验及 Hessian 分析](../small_gicp/README.md)：依赖环境、六自由度 GICP、逐轮特征值与 LM 日志。
- 本目录 [demo.cpp](./demo.cpp) 与 [CMakeLists.txt](./CMakeLists.txt)：独立实验，不替换上述两个示例，也不修改外部 small_gicp 源码。

状态：代码已编写并静态检查，尚未编译或运行。下面列出的是设计、验收条件和待验证实验，不是已经获得的测量结果。

## 1. “给定初值”与“固定平移”

采用 source 到 target 的变换约定：

$$q\simeq Rp+t_0$$

这里 $t_0$ 为外部给定的固定平移列，整个求解中不允许变化。仅将其赋给普通六自由度 GICP 的初始位姿，不能达到这个目的，因为后续更新仍会估计平移。

旋转初值在示例中为单位阵；实际应用可以由 ZYX 顺序的 RPY 构造：

$$R_0=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})$$

求解变量是局部三维旋转增量，不是直接优化三个欧拉角。固定平移也限定了旋转中心的几何含义：先绕 source 原点旋转，再平移。若机构绕其他中心旋转，需要先调整观测模型，参见上面的约束优化笔记。

## 2. GICP 加权残差与右扰动

对当前对应点 $p_i,q_{\pi(i)}$，使用残差和协方差权重：

$$r_i=q_{\pi(i)}-(Rp_i+t_0),\qquad W_i=(C_{q_i}+RC_{p_i}R^\top)^{-1}$$

$$E=\frac12\sum_i r_i^\top W_i r_i$$

复用 small_gicp 的 `GICPFactor`、对应剔除、协方差估计及 KD-tree。这里沿用库的近似：外层线性化更新对应关系和权重，内层 LM 试探时将它们冻结，不对权重的旋转依赖求导。

原 Ceres 示例使用左扰动，本例沿用 small_gicp 的右扰动：

$$R'=R\operatorname{Exp}([\delta\theta]_\times),\qquad t'=t_0$$

对残差 $q-Rp-t_0$，右扰动旋转 Jacobian 为：

$$J_{r,i}=R[p_i]_\times$$

左扰动和右扰动不能直接共用 Jacobian；本例直接提取库已线性化的旋转块，避免重新手写点因子。

## 3. 从六维系统提取三维旋转系统

库的增量顺序是 `[rx ry rz tx ty tz]`。按此顺序分块：

$$H=\begin{bmatrix}H_{rr}&H_{rt}\\H_{tr}&H_{tt}\end{bmatrix},\qquad b=\begin{bmatrix}b_r\\b_t\end{bmatrix}$$

固定平移令平移增量为零，局部二次目标只保留旋转项，因此 LM 求解为：

$$(H_{rr}+\mu I_3)\delta\theta=-b_r$$

本例通过 `Registration` 的第五个模板参数传入自定义 `RotationOnlyLM`。仍计算原始六维信息矩阵以复用库的点因子和并行归约，但只分解、求解三维旋转矩阵。

不要先求六维解再清零后三维：六维旋转步长受到交叉块耦合影响，并非固定平移时的最优步长。这里也不使用消去未知平移的 Schur 补；平移是已知常量，不是待消元的未知量。

候选位姿只更新 `linear()`，`translation()` 始终赋为输入的固定值。`se3_exp([delta_theta, 0])` 仅用来获得旋转指数映射，程序没有求解任何平移增量。

内置 `RestrictDoFFactor` 使用大对角惩罚，仅为软限制；本例的硬固定不依赖惩罚权重大小，也不依赖优化结束后再覆盖平移。

## 4. LM 步长接受与停止条件

每次外层迭代执行：

1. 用当前旋转、固定平移重新匹配和线性化。
2. 检查有效对应数（本 demo 最少 20 个）、有限数值和旋转块谱。
3. 对未加阻尼的 $H_{rr}$ 做特征分解并打印。
4. 解三维阻尼方程，构造候选旋转，用本轮固定对应关系及权重评估误差。
5. 误差有限且不增时接受，阻尼除以 10；否则阻尼乘以 10 后重试。
6. 接受的旋转步长小于 $10^{-7}$ rad 时停止，否则重新匹配。

初始阻尼为 $10^{-3}$，每轮最多 10 次试探，最多 30 次外层迭代。没有可接受步长、对应不足、非有限数值或分解失败会显式报错并返回非零。达到外层上限时保留 `converged=false`，最终自检失败。

这些是教学实验的参数，不是通用生产默认值。阻尼可能让弱约束系统的步长很小，所以“增量收敛”不能单独作为配准正确或可观测的证明；实际应用还需按噪声、残差和几何覆盖制定验收条件。

当前 `GeneralFactor` 使用 `NullFactor`，没有额外先验或鲁棒核。原 Ceres 示例中的 Huber 核不会因迁移而自动保留。若加入额外因子，线性化和候选误差评估必须使用同一个目标函数。

## 5. 三维特征值的解释

$$H_{rr}=V_r\Lambda_r V_r^\top,\qquad \rho_r=\frac{\lambda_{\min}(H_{rr})}{\lambda_{\max}(H_{rr})}$$

每轮打印三个升序特征值、比值和最弱旋转方向 `[rx ry rz]`。它们描述**给定平移条件下**的旋转约束，不是原六自由度问题的完整可观测性。与基础 demo 的六维混合尺度比值不能直接横向比较。

本例在比值小于 $10^{-4}$ 时打印诊断警告，不自动宣称失败或施加先验。整个矩阵尺度也要结合点数、空间分布与协方差权重分析：一个整体很小但比值良好的矩阵并不代表信息充足。最大特征值非正或明显负的最小特征值会报错。特征向量整体符号无区别，多个接近特征值对应的向量可能对扰动敏感。

日志的 Hessian 在每次外层更新前计算，没有包含阻尼。内层试探复用同一矩阵。结果中的 `H`、`b`、内点数保留最后一次线性化信息，并未在最终更新后重新计算；`error` 是该轮固定对应/权重下接受步长后的误差。

自定义优化器将 `iterations` 定义为实际外层次数，从 1 开始计数。这与当前上游优化器存储最后一次零起始编号的行为不同，输出标签明确写为 `outer iterations (count)`。

## 6. 两组实验与自检

合成点云与基础 demo 一致：三个互相垂直的平面，真值为：

$$R_{true}=R_z(0.08)R_y(-0.05),\qquad t_{true}=(0.12,-0.08,0.06)^\top\ \mathrm{m}$$

预处理使用 0.05 米体素、10 个协方差邻点，最大对应距离为 0.5 米；使用单线程降低并行降采样导致的运行间差异。

| 实验 | 输入固定平移 | 自检要求 | 观察重点 |
| --- | --- | --- | --- |
| 正确平移 | 真值平移 | 收敛、平移漂移严格为零、旋转误差小于 $10^{-3}$ rad、位姿有限 | 是否恢复已知旋转 |
| 有偏平移 | 真值 x 增加 0.02 米 | 收敛、平移漂移严格为零、位姿有限 | 残余误差与旋转偏差，不要求恢复真值旋转 |

两组都从单位旋转开始。运行时打印旋转测地误差：

$$e_R=\operatorname{angle}(R_{true}^\top R_{estimated})$$

还利用原始合成点的已知索引对应计算独立诊断：

$$\mathrm{RMSE}_{known}=\sqrt{\frac1N\sum_i\|R_{estimated}p_i+t_0-q_i\|^2}$$

此 RMSE 不参与优化，不能与加权 GICP 目标 `e` 当成同一量。真实无对应点云无法直接照搬这项指标。

平移有偏差时，旋转可能部分代偿误差，但不保证所有场景都会出现明显旋转偏差；本例对称几何尤其可能降低这种代偿。无论如何，优化器不会修正被固定的平移。若平移不可靠，应考虑外层候选平移搜索，或六自由度优化加平移先验，这两者都不是当前严格仅旋转问题。

自检均使用运行时判断，在 Release 的 `NDEBUG` 下也有效。尚未执行这两组实验，不能预先声称它们通过。没有覆盖真实扫描、部分重叠、离群点、单平面退化或大初始旋转。

## 7. 构建与运行

基础依赖沿用 small_gicp 实验：C++17、Eigen、可选 OpenMP。没有引入 Ceres 或 PCL。默认外部源码目录为本机已有的 `/home/zzh/small_gicp-master/small_gicp`，可通过 `SMALL_GICP_DIR` 覆盖。

CMake 把外部 `registration.cpp` 与 `registration_helper.cpp` 编译为本实验静态库，因为 `preprocess_points` 仍需要 helper 实现。不会安装库或修改外部源码。

在知识库根目录执行（需要单独编译授权）：

```bash
cmake -S 03-Knowledges/标定/仅优化旋转的gicp \
  -B /tmp/rotation-only-gicp-build \
  -DSMALL_GICP_DIR=/home/zzh/small_gicp-master/small_gicp \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/rotation-only-gicp-build -j2
/tmp/rotation-only-gicp-build/rotation_only_gicp_demo
```

返回 0 表示两组运行时自检通过，返回 1 表示求解异常或至少一项检查失败。构建产物放在 `/tmp`，实验日志和分析可以追加在本目录。

## 8. 源码对应位置

以下均指本机外部 small_gicp 仓库内的相对路径，可与实际版本对照：

- `include/small_gicp/factors/gicp_factor.hpp`：加权残差、右扰动 Jacobian 和误差评估。
- `include/small_gicp/registration/registration.hpp`：可替换 Optimizer 的模板接口。
- `include/small_gicp/registration/optimizer.hpp`：原始六自由度 LM 流程。
- `include/small_gicp/factors/general_factor.hpp`：`NullFactor` 与软约束 `RestrictDoFFactor`。
- `src/small_gicp/registration/registration_helper.cpp`：预处理和默认 GICP 调用关系。

项目入口：[koide3/small_gicp](https://github.com/koide3/small_gicp)。本文实现依据本机源码接口，不假定未来版本接口完全相同。
