# 计划：ICP 中「只优化旋转」与「平移球约束」的 Ceres 实现文档 + Demo

## 1. 目标

在 `03-Knowledges/标定/带约束的位姿优化/` 下产出一篇中文算法文档与两个可独立编译的 Ceres demo，覆盖两种带约束的 ICP 位姿优化：

- **问题 A（xyz 已知）**：齐次变换 `T = [R | t]` 中 `t` 完全已知且固定，只优化旋转（用 rpy 参数化）。
- **问题 B（平移球约束）**：`‖t − t₀‖ ≤ r`（`r = 0.1 m`），旋转无约束无阈值。

文档必须明确回答用户的第 2 问：**问题 B 与问题 A 在算法上有哪些实质区别**（自由度、可观测性、闭式解是否存在、退化模式、收敛行为）。

## 2. 已确认的决策（不要在实现时改动）

| 项 | 决定 |
| --- | --- |
| 依赖 | 仅 Eigen3 + Ceres，自包含；点云用 `std::vector<Eigen::Vector3d>`，最近邻用暴力搜索；不引入 PCL / nanoflann |
| 文件结构 | 一篇文档 + 两个 `.cpp` + 一个 `CMakeLists.txt` |
| 残差形式 | 只做 point-to-point，不做 point-to-plane |
| `xyz` 语义 | 齐次变换矩阵的平移列，模型为 `q ≈ R·p + t`，旋转中心为 source 坐标系原点 |
| 约束几何 | **球约束** `‖t − t₀‖ ≤ 0.1`，不是逐轴 ±10 cm 的盒 |
| 球约束实现 | **平滑重参数化为主方案**；文档兼讲「盒约束 + 球惩罚残差」与「外层投影法」两种备选 |
| 闭式解对照 | 需要；在问题 A 的 demo 中用正交 Procrustes/SVD 做数值自检 |
| 旋转参数块 | **全局 rpy**（参数块即 `rpy[3]`，`R = R_z(yaw)R_y(pitch)R_x(roll)`）；增量 rpy 方案只作为文档中的代码片段，不进 demo |

## 3. 环境事实（已在本机核实）

- Ceres 版本：`2.0.0`（`/usr/include/ceres/version.h:35`）。
- `ceres::Problem::SetParameterLowerBound / SetParameterUpperBound` 存在（`/usr/include/ceres/problem.h:333`），但**只能表达逐分量 bound**，无法直接表达球约束。
- **禁止使用 `ceres::EulerAnglesToRotationMatrix`**（`/usr/include/ceres/rotation.h:428`）：它的输入是**度**，且 `euler[0]` 被命名为 pitch，其角色映射与工程常用 roll-pitch-yaw 约定不一致。两个 demo 都必须自己写 `rpy → R`。
- 两个 demo 的参数块均为欧氏向量，**不需要 `LocalParameterization`**，因此不受 Ceres 2.1+ `Manifold` 迁移影响。文档在讨论「四元数替代方案」时必须标注该 API 的版本差异（2.0 用 `QuaternionParameterization`，2.1+ 用 `QuaternionManifold`）。
- 仓库现有文档风格（参考 `03-Knowledges/标定/单一相机标定内参外参/张正友标定法及工程细节.md`）：YAML frontmatter（`title/date/tags`）+ 中文正文 + `$$ ... $$` 公式块 + 表格。新文档保持一致。
- 目标文件 `03-Knowledges/标定/带约束的位姿优化/只优化旋转rpy.md` 目前为 0 字节，可直接写入，无覆盖风险。该目录下无任何已有代码文件。
- CMake 导入目标已核实：`Ceres::ceres`（`/usr/lib/cmake/Ceres/CeresTargets.cmake:63`）自带 `INTERFACE_COMPILE_FEATURES "cxx_std_14"`，且其 `INTERFACE_LINK_LIBRARIES` 已包含 `Eigen3::Eigen`（同文件 `:67`、`:69`）。`Eigen3::Eigen` 由 `/usr/share/eigen3/cmake/Eigen3Targets.cmake:54` 提供。因此只链接 `Ceres::ceres` 即可拿到 Eigen 头路径与 C++14；`CeresConfig.cmake:271` 另外把 `CERES_LIBRARIES` 设为 `Ceres::ceres`。

## 4. 待创建文件

全部位于 `03-Knowledges/标定/带约束的位姿优化/`：

1. `只优化旋转rpy.md` —— 算法原理文档（两个问题合一篇）
2. `demo_rotation_only.cpp` —— 问题 A
3. `demo_sphere_constrained.cpp` —— 问题 B
4. `CMakeLists.txt` —— 两个可执行目标

## 5. 任务清单

### 任务 1：写 `只优化旋转rpy.md`

按以下章节组织，公式用 `$$` 块：

**1) 问题设定与记号**
- 模型 `q ≈ R·p + t`，`R ∈ SO(3)`，`t ∈ R³`。
- 明确 `T = [R | t; 0 1]` 把 source 点变换到 target 系；`t` 即该矩阵的平移列。
- 明确旋转中心是 source 系原点——这一点决定「固定 t」到底约束了什么。

**2) rpy 参数化与其代价**
- 采用 `R(φ,θ,ψ) = R_z(ψ)·R_y(θ)·R_x(φ)`，写出 3×3 展开式，并声明单位为**弧度**。
- 明确指出 Ceres 2.0 内置 `EulerAnglesToRotationMatrix` 用度且命名不同，故自行实现。
- 万向锁：`θ → ±90°` 时 `φ` 与 `ψ` 退化为同一自由度，Jacobian 掉秩。给出两种规避方案：
  - 增量 rpy：`R = R₀ · R_z(δψ)R_y(δθ)R_x(δφ)`，优化量始终在 0 附近，收敛后再把 `R` 转回全局 rpy 输出。**此处给出可直接粘贴的 functor 代码片段**（约 15 行），说明每轮求解后把增量吸收进 `R₀` 并重置 `δ=0`；demo 不采用此方案，故必须在片段旁注明"与 demo 的全局 rpy 版本二选一"。
  - 改用角轴/四元数参数化（附 Ceres 2.0 vs 2.1+ API 差异说明）。
- 结论：rpy 便于对外表达和加角度阈值，但作为**优化变量**在近奇异姿态下不安全。demo 选择全局 rpy 是为了贴合"只优化 rpy"的字面需求与可读性，并通过把 `pitch_gt` 取在远离 ±90° 处规避奇异；生产环境若姿态范围可能扫过 pitch ≈ ±90°，应切换到增量方案或四元数。

**3) 问题 A：固定 t，只优化旋转**
- 目标函数：

  $$
  \min_{R\in SO(3)}\ \sum_i \rho\!\left(\left\|R\,p_i + t - q_{\pi(i)}\right\|^2\right)
  $$

  其中 `t` 为常量，`π(i)` 为当前最近邻对应，`ρ` 为 Huber 核。
- 残差为 3 维向量残差（不要写成平方和标量），参数块只有 `rpy[3]`。
- ICP 外层迭代结构：`建立对应 → 距离/互近邻剔除 → Ceres 求解 → 更新 R → 重建对应`，直到 `R` 增量小于阈值或达到最大轮数。
- **闭式解（必须写清一个易错点）**：固定 `t` 后令 `q'_i = q_i − t`，则

  $$
  H=\sum_i q'_i\,p_i^{\top},\quad H=U\Sigma V^{\top},\quad
  R^\star=U\,\mathrm{diag}(1,1,\det(UV^{\top}))\,V^{\top}
  $$

  强调：**这里不能减质心**。标准 Kabsch/ICP 之所以先减质心，前提是 `t` 自由可吸收质心差；本问题 `t` 已固定，减质心等价于偷偷放开了平移自由度，会得到错误的 `R`。
- 既然有闭式解，为什么还用 Ceres：Huber 等鲁棒核、点权重、额外先验/正则项、与其他传感器残差联合求解、角度软约束——这些闭式解都无法容纳。
- 可观测性与退化：source 点集近似球对称、或全部点近似落在过原点的一条直线/一张过原点的平面上时，旋转不可完全确定；给出判断手段（Ceres `Covariance` 或残差 Hessian 的条件数/最小特征值）。
- **`t` 的精度直接决定精度上限**：`t` 有误差时，由于它不可调，误差会被旋转"代偿"，产生系统性旋转偏差，表现为收敛后的残差 RMS 存在不可下降的下限。文档要给出这个诊断信号。

**4) 问题 B：旋转自由 + 平移球约束**
- 约束 `‖t − t₀‖ ≤ r`。先说明 Ceres 的 bound 约束是逐分量的，**不能**直接表达球约束。
- 主方案——平滑重参数化（无约束化）：

  $$
  t(s) = t_0 + r\,\frac{s}{\sqrt{1+\|s\|^2}},\qquad s\in\mathbb{R}^3
  $$

  性质：对任意 `s` 恒有 `‖t−t₀‖ < r`（严格不可能违反约束）；`s = 0 ⟺ t = t₀`，且此处 `∂t/∂s = r·I`，条件数良好，适合作为初值；映射光滑，autodiff 可直接求导。
  代价：边界不可达（渐近趋近）。若真解落在边界上，`s` 会发散、收敛变慢。因此**必须输出 `‖t−t₀‖ / r`**；当其 > 0.99 判定约束活跃，此时应报告"约束饱和"而不是当作正常收敛，并考虑重新给定 `t₀` 或改用备选方案。
- 备选方案一（盒 + 球惩罚）：先用 `SetParameterLowerBound/UpperBound` 把 `t` 每个分量限制在球的外接立方体 `[t₀ᵏ−r, t₀ᵏ+r]`，再加一条残差 `w·max(0, ‖t−t₀‖−r)` 把解拉回球内。优点是边界可达、收敛行为可预测；缺点是软约束，允许微小越界，且 `w` 需调。
- 备选方案二（外层投影）：每轮 Ceres 求解后若越界就把 `t` 投影回球面再重新建立对应。实现最简，但不是真正的约束求解，可能在边界附近振荡。
- 若用户实际需求是**逐轴 ±10 cm 的盒**（用户原话的字面读法），说明这是最简单的情形：`SetParameterLowerBound/UpperBound` 三行即可，无需重参数化；并提醒若盒是沿传感器/初值姿态轴向而非世界轴向，需先重参数化 `t = t₀ + R_box·s` 再对 `s` 加 bound。

**5) 两个问题的区别（直接回答用户第 2 问）**

用表格对比，至少覆盖：

| 维度 | 问题 A（t 固定） | 问题 B（t 球约束） |
| --- | --- | --- |
| 自由度 | 3 | 6（其中 3 个被不等式约束） |
| 平移-旋转耦合 | 无，t 不参与优化 | 存在，经典 ICP 耦合退化全部回归 |
| 退化模式 | 仅旋转方向的退化 | 额外出现平面沿面内滑动、圆柱沿轴滑动、球心任意等滑动退化 |
| 闭式解 | 有（不减质心的 Procrustes/SVD） | 无约束情形是 Kabsch（减质心）；带球约束无简单闭式解，**这才是真正必须用 Ceres 的场景** |
| 约束是否常活跃 | 恒定活跃（硬固定） | 通常不活跃；若初值和对应关系正常，结果≈无约束 6-DOF ICP，约束只起"防跑飞"护栏作用 |
| 数值尺度 | 单一尺度（弧度） | 弧度与米混合，需关注参数缩放与 `d_max`、Huber `δ` 的取值 |
| 对初值的敏感度 | 对 `t` 的精度极敏感 | 对 `t₀` 的精度要求放宽到 `r` 量级 |

结论句要写明：**约束从"等式固定"变成"不等式范围"后，算法结构（外层 ICP + 内层 Ceres）不变，变的是参数块、退化谱系和闭式解的可用性**；因此问题 B 不能简单地在问题 A 上加个 bound 了事，需要按上表重新评估退化与初值策略。

**6) 工程参数与验收**
- `d_max` 由粗到细的衰减策略；Huber `δ` 与点云噪声的关系；最小对应点数；收敛判据（旋转增量 `deg`、平移增量 `m`、残差相对下降）。
- 精度评估：旋转用测地距离 `θ = arccos((tr(R_gtᵀR)−1)/2)`，平移用 `‖t−t_gt‖`；不要只看残差 RMS。

**7) 参考链接**
- Ceres bound 约束与 `Problem` API、`LossFunction`、`AutoDiffCostFunction`、`Covariance` 的官方文档链接；Kabsch/正交 Procrustes 的原始文献。按仓库既有风格把链接内嵌在正文中。

### 任务 2：写 `demo_rotation_only.cpp`（问题 A）

结构要求：

1. `RpyToRotation<T>(const T* rpy, Eigen::Matrix<T,3,3>& R)` —— `R_z(yaw)·R_y(pitch)·R_x(roll)`，弧度，模板化以支持 `ceres::Jet`。
2. `RotationToRpy(const Eigen::Matrix3d&) -> Eigen::Vector3d` —— 仅用于结果输出与和闭式解比对。必须与 `RpyToRotation` 的 `Z·Y·X` 次序严格互逆（`pitch = asin(-R(2,0))`、`roll = atan2(R(2,1),R(2,2))`、`yaw = atan2(R(1,0),R(0,0))`），并加上 `pitch` 近 ±90° 的退化分支。
3. 合成数据生成：固定种子 `std::mt19937 rng(42)` 保证可复现。source 点云用三个互相垂直的平面片组合（避免旋转不可观测）；给定 `rpy_gt`、`t_gt`；由 `q = R_gt·p + t_gt + 高斯噪声` 生成 target；再打乱 target 顺序，使最近邻搜索确实在工作。
4. `struct PointToPointRotationOnly`：成员 `Eigen::Vector3d src_, dst_, t_fixed_`；`operator()(const T* rpy, T* residual)` 输出 3 维残差 `R(rpy)·src + t_fixed − dst`。用 `ceres::AutoDiffCostFunction<..., 3, 3>`。
5. 外层 ICP 循环（`kMaxOuterIter`）：暴力最近邻 + `d_max` 剔除 + 互近邻校验；每轮新建 `ceres::Problem`，`AddResidualBlock(cost, new ceres::HuberLoss(delta), rpy)`；`DENSE_QR`；打印每轮对应点数与残差 RMS；按旋转增量小于阈值提前退出。
6. **闭式解自检（独立于主 ICP 流程，用严格可比的条件）**：单独构造一组**无噪声、对应关系已知**的点对，然后分别求解：
   - Ceres（**不加 Huber**，`nullptr` 作为 loss）；
   - `H = Σ (q_i − t)·p_iᵀ` 的 SVD 解（**不减质心**），`R = U·diag(1,1,det(UVᵀ))·Vᵀ`。

   打印两者测地角差，应在 `1e-8` 量级。必须在注释中说明：主 ICP 流程带 Huber 与噪声时二者本就不该完全相等（Huber 是加权的、SVD 是等权的），所以自检必须用这组独立的干净数据，不能直接拿主流程末轮的结果比。
7. 额外打印一个反面对照：对同一组干净数据用**减质心**版本的 SVD 求解，打印其与 `R_gt` 的测地角误差，用数据证明"固定 t 时不能减质心"。为了让该误差明显可见，`t_gt` 不要取零，且 source 点云质心不要取在原点。
8. 打印主 ICP 结果与 `rpy_gt` 的测地角误差。
9. 所有魔数集中为文件顶部 `constexpr`，便于改参数复现。
10. `main` 开头调用 `google::InitGoogleLogging(argv[0])`（Ceres 2.0 依赖 glog，见 `CeresTargets.cmake:69`），避免运行时 glog 告警。

### 任务 3：写 `demo_sphere_constrained.cpp`（问题 B）

1. 复用同样的 `RpyToRotation` / `RotationToRpy` / 数据生成（**直接复制，保持单文件自包含**，不抽公共头文件）。
2. `struct PointToPointSphereBounded`：参数块两个 —— `rpy[3]` 与 `s[3]`；内部按 `t = t0 + r·s/sqrt(1+‖s‖²)` 计算 `t`，残差 `R(rpy)·src + t − dst`。`ceres::AutoDiffCostFunction<..., 3, 3, 3>`。注意 `‖s‖²` 要用 `s[0]*s[0]+...` 显式写，避免 Eigen `Map` 与 `Jet` 的混用问题。
3. `s` 初值全零（即 `t = t₀`）；提供 `SToT(const double* s)` 辅助函数供外层打印当前 `t`。
4. 外层 ICP 循环同上；每轮打印 `‖t − t₀‖`、`‖t−t₀‖/r` 与 `‖s‖`。
5. 两个场景各跑一遍并打印对比（同一函数 `RunScenario(...)` 调两次）：
   - 场景 1：`t_gt` 在球内（`‖t_gt − t₀‖ = 0.04 m`）→ 期望约束不活跃，`t` 收敛到 `t_gt`，比值明显小于 1；
   - 场景 2：`t_gt` 在球外（`‖t_gt − t₀‖ = 0.30 m`）→ 期望约束饱和，比值趋近 1，`‖s‖` 显著增大，打印"约束活跃/饱和"告警，并说明此时旋转会被迫代偿平移误差。
6. 在文件末尾以注释块给出备选实现片段（不参与编译）：直接把 `t[3]` 作为参数块并用 `SetParameterLowerBound/UpperBound` 实现逐轴 ±10 cm 盒约束的写法，便于用户按字面需求切换。
7. 打印旋转测地角误差与平移误差 `‖t − t_gt‖`。
8. `main` 开头同样调用 `google::InitGoogleLogging(argv[0])`。

### 任务 4：写 `CMakeLists.txt`

- `cmake_minimum_required(VERSION 3.10)`，`project(constrained_icp_demo CXX)`。
- `find_package(Ceres REQUIRED)`。**不需要**单独 `find_package(Eigen3)`，因为 `Ceres::ceres` 的 `INTERFACE_LINK_LIBRARIES` 已含 `Eigen3::Eigen`，且它自带 `cxx_std_14`；但为可移植性仍显式写 `find_package(Eigen3 REQUIRED)` 并链接，两者都链接不会冲突。
- 两个可执行目标 `demo_rotation_only`、`demo_sphere_constrained`，`target_link_libraries(... PRIVATE Ceres::ceres Eigen3::Eigen)`。
- 显式 `target_compile_features(... PRIVATE cxx_std_14)`，不依赖全局 `CMAKE_CXX_STANDARD`。
- 注释中给出旧版 Ceres 的回退写法（`${CERES_INCLUDE_DIRS}` + `${CERES_LIBRARIES}`）。
- 文件顶部注释写明构建命令，但**不在本次任务中执行**。

### 任务 5：收尾

1. `git status` / `git diff --stat` 确认只新增了上述 4 个文件，没有触碰其他文件。
2. 用只读方式复查两个 `.cpp` 的模板化是否对 `ceres::Jet` 安全（禁止出现 `double` 与 `T` 混算、禁止对 `T` 调用 `std::sqrt` 之外的非模板函数、`Eigen::Matrix<T,3,3>` 初始化正确）。
3. 汇报改动清单，并**单独申请编译授权**，附：工作目录、完整命令、用途、目标平台、生成文件、风险。
4. 明确声明：未执行编译；按全局规则编译需用户单独确认。

## 6. 风险与易错点

- **减质心陷阱**（问题 A 最大的坑）：固定 `t` 时做 Kabsch 式减质心会得到错误旋转。文档和 demo 都要显式演示这个错误。为让错误可见，`t_gt ≠ 0` 且 source 质心 ≠ 原点。
- **闭式解自检的可比性**：带 Huber 的 Ceres 解与等权 SVD 解本就不相等。自检必须用独立的无噪声、已知对应、无 Huber 的数据组，否则会得到"两者不一致"的假阳性结论。
- **`EulerAnglesToRotationMatrix` 单位与命名陷阱**：Ceres 2.0 该函数用度，且第一个分量叫 pitch。必须自己实现 rpy→R。
- **`RotationToRpy` 必须与 `RpyToRotation` 同次序互逆**：次序写错会让"角误差"数字看着合理但语义错误。建议在 demo 里加一条往返自检（`rpy → R → rpy` 误差 < 1e-12）。
- **球约束边界不可达**：重参数化是严格内点法，真解在边界时不收敛到边界。必须输出 `‖t−t₀‖/r` 作为约束活跃度指标，否则会把"卡在边界"误读为"收敛正常"。
- **autodiff 类型污染**：`t_fixed_` 是 `double`，与 `T` 运算时需 `.cast<T>()` 或 `T(...)` 显式转换；`sqrt` 要靠 ADL 解析到 `ceres::sqrt`（写 `using std::sqrt;` 或直接 `sqrt(...)`，不要写 `std::sqrt`）。
- **暴力最近邻复杂度**：`O(N·M)`，demo 点数控制在 2000 量级以内，并在注释中说明生产环境应换 KD-tree。
- **万向锁**：demo 的 `rpy_gt` 的 pitch 不要取接近 ±90°，否则会观察到病态收敛；文档需说明这不是实现 bug 而是参数化固有缺陷。
- **glog 初始化**：Ceres 2.0 链接 glog，未初始化时会有运行期告警。两个 demo 的 `main` 都要调 `google::InitGoogleLogging`。

## 7. 验收标准

- 文档明确回答两个问题，且第 2 问有独立的"区别"章节与独立的 Ceres 算法（不是在问题 A 上加一句 bound）。
- 问题 A 的 demo 中，在独立的无噪声、已知对应、无 Huber 数据组上，Ceres 解与**不减质心**的 SVD 闭式解一致到 `1e-8` 量级；**减质心**版本对同组数据明显偏离 `R_gt`（角误差应达到肉眼可辨的度级）。
- 问题 A 的 demo 中 `rpy → R → rpy` 往返误差 < `1e-12`。
- 问题 B 的 demo 中，球内场景恢复 `t_gt`（误差在噪声量级）且 `‖t−t₀‖/r` 明显小于 1；球外场景 `‖t−t₀‖/r` 趋近 1 并打印约束饱和告警。
- 两个 demo 均只依赖 Eigen + Ceres（+ Ceres 自身依赖的 glog），不引入 PCL / nanoflann。
- 两个 demo 使用固定随机种子，重复运行输出一致。
- 未经用户单独授权，不执行任何编译命令。

## 8. 显式排除范围

- point-to-plane 残差、法向量估计（文档不展开，仅在"后续可扩展"处一句话提及）。
- 真实点云文件 IO（PCD/PLY 读取）。
- KD-tree 实现。
- 角度阈值约束的实现（用户明确说明问题 B 中角度无阈值）。
