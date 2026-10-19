gi[s01]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/src/example/01_basic_registration.cpp
[s02]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/src/example/02_basic_registration_pcl.cpp
[s03]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/src/example/03_registration_template.cpp
[helper]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/src/small_gicp/registration/registration_helper.cpp
[setting]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/registration/registration_helper.hpp
[pclimpl]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/pcl/pcl_registration_impl.hpp
[registration]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/registration/registration.hpp
[optimizer]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/registration/optimizer.hpp
[criteria]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/registration/termination_criteria.hpp
[factor]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/factors/gicp_factor.hpp
[normal]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/util/normal_estimation.hpp
[voxel]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/ann/gaussian_voxelmap.hpp
[ivoxel]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/ann/incremental_voxelmap.hpp
[odometry]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/src/example/kitti_odometry.py
[cmake]: https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/CMakeLists.txt

# small_gicp 示例学习：接口、缓存复用与标定精度

本文结合 `01_basic_registration.cpp`、`02_basic_registration_pcl.cpp`、`03_registration_template.cpp` 和实际实现，回答四个问题：是否需要 PCL、为什么交换 source/target、VGICP 的完整流程，以及好初值能给标定带来什么。

分析依据为本机 `/home/zzh/small_gicp-master/small_gicp`，commit `fa0cfc983417c044b6036f71dfa448038bb9c820`。下文源码链接固定到该提交；“工程建议”是基于源码与问题模型的推论，不是性能实测结论。本次仅整理文档，未重新编译或跑精度对比。

相关知识：[基础实验与 Hessian 日志](./README.md)、[仅优化旋转的 GICP](../仅优化旋转的gicp/README.md)、[带约束的位姿优化](../带约束的位姿优化/只优化旋转rpy.md)。

## 1. 01 与 02 的差别：接口、数据结构和算法要分开

### 1.1 四种用法对应关系

| 示例入口         | 点云表示                                  | 预处理由谁组织                       | 算法/搜索                   | 谁管理缓存                   |
| ---------------- | ----------------------------------------- | ------------------------------------ | --------------------------- | ---------------------------- |
| `01::example1` | `std::vector<Eigen::Vector4f>`          | `align` helper 内部处理            | 默认 GICP / KD-tree         | 每次输入原始数组会重复预处理 |
| `01::example2` | 先转为 small_gicp 点云                    | 显式`preprocess_points`            | GICP / KD-tree              | 调用者保留预处理结果         |
| `02::example1` | `pcl::PointCloud<pcl::PointXYZ>`        | 调用者降采样，PCL 适配层补齐协方差等 | 显式选 VGICP / 高斯体素地图 | `RegistrationPCL` 内部缓存 |
| `02::example2` | `pcl::PointCloud<pcl::PointCovariance>` | 显式降采样、协方差、KD-tree          | GICP / KD-tree              | 调用者管理                   |
| `03::example1` | `small_gicp::PointCloud`                | 显式调用库的各个步骤                 | GICP / KD-tree              | 调用者管理                   |

来源：[01 示例][s01]、[02 示例][s02]、[03 示例][s03]、[helper 实现][helper]。

因此，“不调用 PCL 接口就要全部自己写点云处理”并不成立。`03` 手动组织的是调用顺序，降采样、协方差估计和 KD-tree 仍由 small_gicp 实现。只有特殊点类型或搜索策略才需要额外适配。`03` 后面的自定义 traits/暴力搜索是扩展示范，不能据此认为默认实现用暴力搜索。[03 模板示例源码][s03]

还要区分 `small_gicp::RegistrationPCL` 与 `pcl::GeneralizedIterativeClosestPoint`：前者继承 PCL 接口，但内部仍调用 small_gicp 的 `Registration<GICPFactor, ParallelReductionOMP>`。本次运行并不是“small_gicp 算法对比 PCL 原生 GICP 算法”。[PCL 适配实现][pclimpl]

### 1.2 不依赖 PCL 的优势与代价

以下为接口和实现带来的工程判断：

- **依赖更少**：只需要点云配准时，可避免为 PCL 集成额外模块及其传递依赖。本机官方 `BUILD_EXAMPLES=ON` 会启用 PCL/TBB 并查找 fmt；这属于上游示例构建配置，不代表核心算法必须依赖这些库。[官方构建配置][cmake]
- **数据转换可以减少**：原始数据已经是 Eigen 数组或自定义容器时，不必为了接口再转成 `pcl::PointCloud`；但 helper 本身仍可能转换、复制、降采样，不能声称全程零拷贝。[helper 实现][helper]
- **预处理复用更显式**：调用者能保留目标云、协方差和树，在多次初值尝试或连续配准时复用，而不是每次走原始数组的 `align` 重算。[01 示例源码][s01]
- **更容易定制优化器和因子**：例如严格固定平移、仅求三维旋转，就适合 `Registration` 模板接口；但这不排斥 PCL 点云，`02::example2` 已证明 PCL 数据也能直接进入该接口。[02 PCL 示例源码][s02][registration][registration]

相应地，自己管理预处理需要负责数据生命周期、缓存有效性、异常输入和参数一致性。没有必要重写已经成熟的文件读取、滤波或邻域搜索。

### 1.3 PCL 的“劣势”不能泛化成低精度或低性能

PCL 的成本主要可能来自依赖规模、模板编译、数据转换和封装可控性；是否形成实际瓶颈必须测量。如果工程本来就用 PCL 读写、滤波、分割，复用已有点云通常更直接。

在此适配实现中，优化内部使用 double，而最终 `getFinalTransformation()` 转为 float。可以通过 `getRegistrationResult().T_target_source` 取得内部 double 位姿。但 `01` 的示例输入同样是 float 坐标，转成 double 不会恢复输入已经丢失的信息。因此不能把两份示例的误差差异简单归因于 PCL 或浮点精度。[01 示例源码][s01][pclimpl][pclimpl]

公平比较需要统一算法类型、输入点集合、协方差邻域、阈值、初值、停止条件、线程数，并区分预处理时间与迭代时间。直接比较原样 `01::example1` 与 `02::example1`，实际上同时改变了 GICP/VGICP、邻域及默认阈值。

## 2. 反向实验真正演示的是缓存复用

### 2.1 同一对点云的示例动作

`02::example1` 先以 A 为 target、B 为 source，求 $T_{A\leftarrow B}$，随后执行：

```cpp
reg.swapSourceAndTarget();
reg.align(*aligned);
// 为便于与第一次比较，将反向结果取逆后输出。
```

交换后解的是 $T_{B\leftarrow A}$。打印它的逆只是统一输出方向；不是把第一次结果当初值继续优化，也不是把两个结果融合。该示例未显式给第二次 `align` 传初值。[02 PCL 示例源码][s02]

`swapSourceAndTarget()` 实际交换四组对象：输入点云、KD-tree、协方差、GaussianVoxelMap。已经计算好的数据跟随点云一起切换角色。[PCL 适配实现][pclimpl]

### 2.2 连续帧中的真实用法

假设按“上一帧 target、当前帧 source”做 scan-to-scan 里程计：

| 阶段          | target      | source      | 动作                                  |
| ------------- | ----------- | ----------- | ------------------------------------- |
| 第 k 次配准   | $P_{k-1}$ | $P_k$     | 求$T_{k-1\leftarrow k}$             |
| 交换后        | $P_k$     | $P_{k-1}$ | $P_k$ 的缓存整体成为下一轮 target   |
| 下一帧到达    | $P_k$     | $P_{k+1}$ | 设置新 source，仅为新帧建立预处理数据 |
| 第 k+1 次配准 | $P_k$     | $P_{k+1}$ | 求$T_{k\leftarrow k+1}$             |

概念流程：

```text
用上一帧缓存和当前帧进行配准
检查结果是否可接受
累计位姿
swapSourceAndTarget()              # 当前帧及其缓存成为 target
setInputSource(下一帧降采样点云)    # 替换旧 source
align(输出点云, 下一次相对运动预测)
```

**真实里程计不需要交换后立即对旧的两帧再算一次反向配准。** 示例之所以这样写，是没有第三帧，利用反向调用展示“交换后仍可直接复用数据”。这是对示例用途的解释；缓存交换本身由源码直接确认。[02 PCL 示例源码][s02][pclimpl][pclimpl]

官方 Python scan-to-scan 示例也将当前帧预处理结果保存为下一轮 target，没有对同一对帧重复正反求解。[官方里程计示例][odometry]

### 2.3 复用的是几何数据，不是下一帧的初值

位姿累计满足：

$$
T_{W\leftarrow k}=T_{W\leftarrow k-1}T_{k-1\leftarrow k}
$$

下一次相对初值可由 IMU、轮速或恒速模型预测。直接沿用上次相对位姿是一种短时运动近似，不是严格几何恒等式。交换缓存不会自动把上一次结果转成新一对帧的正确初值。[官方里程计示例][odometry]

如果 target 是固定地图或关键帧，则通常保留 target，持续替换 source；不必每次交换。切换关键帧时再更新缓存。若点坐标、点序或协方差配置改变，旧缓存不能继续使用。尤其此封装在 `setInputSource/Target` 收到相同指针时直接返回：不要原地改动点云内容后指望重新设置同一指针就触发重建。[PCL 适配实现][pclimpl]

工程上也可以单独做正反向一致性检查：

$$
T_{A\leftarrow B}T_{B\leftarrow A}\approx I
$$

但那属于额外质量检查，会增加一次配准开销；不是里程计必须步骤。最近邻、剔除、降采样和体素聚合都可能造成方向不对称，所以正反结果不完全互逆并不自动证明实现错误，也不能证明其中任一方向就正确。

## 3. VGICP 并非“只走开头”：两者都完整优化

### 3.1 公共流程

本例中的两条路径都包含降采样、局部协方差估计、对应搜索、残差与 Jacobian 构造、Hessian 归约及 LM 迭代。VGICP 不等于预处理算法，也不要求先 VGICP 再 GICP。[PCL 适配实现][pclimpl]

GICP 的局部目标为：

$$
r_i=q_{\pi(i)}-(Rp_i+t),\qquad W_i=(C_{q_i}+RC_{p_i}R^\top)^{-1}
$$

$$
E=\frac12\sum_i r_i^\top W_i r_i
$$

本实现外层重新匹配和计算权重，内层试探冻结本轮对应及权重；不是对协方差权重关于旋转的完整导数求解。[GICP 因子][factor][optimizer][optimizer]

### 3.2 不同的是目标表示和搜索

| 方面     | 当前普通 GICP  | 当前 VGICP 分支              |
| -------- | -------------- | ---------------------------- |
| target   | 带协方差的点云 | 带均值/协方差的高斯体素地图  |
| source   | 带协方差的点云 | 仍为带协方差的点云           |
| 对应搜索 | KD-tree 最近邻 | 哈希体素定位及配置的邻域搜索 |
| 配准因子 | `GICPFactor` | 仍为`GICPFactor`           |
| 优化变量 | 默认六自由度   | 默认六自由度                 |
| 额外参数 | 无高斯体素尺寸 | 高斯体素尺寸及搜索邻域       |

这里应按当前源码理解，不能把所有名为 VGICP 的实现都视作完全相同。[PCL 适配实现][pclimpl][voxel][ivoxel]

### 3.3 此版本中的两个细节

第一，高斯体素的均值是体素内点均值，协方差是插入点的局部协方差经坐标变换后的平均：

$$
\bar p_v=\frac1{N_v}\sum_{i\in v}p_i,\qquad \bar C_v=\frac1{N_v}\sum_{i\in v}C_i
$$

这并不是重新计算体素内点相对均值的散布协方差；不能直接把该实现解释为普通 NDT 体素拟合。[高斯体素聚合][voxel]

第二，`IncrementalVoxelMap` 默认 `search_offsets=1`，只查查询点所在体素；可支持 7、27 体素邻域，但当前 PCL 示例没有调整。找不到该体素时可能没有对应，`max_correspondence_distance=1.0` 不会自动扩展成 1 米球内的所有体素搜索。[体素搜索][ivoxel][pclimpl][pclimpl]

### 3.4 优势与代价

工程推论：体素聚合减少了目标代表元素，哈希定位可以降低每轮搜索成本；地图结构适合增量插入和 scan-to-model 复用。因此可能改善吞吐量，尤其是重复查询较大目标地图时。一次性构建体素也有成本，不能仅凭数据结构就保证每一对小点云更快。[高斯体素聚合][voxel][ivoxel][odometry]

代价是改变了匹配的目标表示及离散搜索范围。大体素可能混合不同表面或削弱细节，边界与搜索邻域也会影响对应；小体素则可能稀疏、空体素增多。精度不一定单调随体素缩小而提升。

`02::example1` 的 0.25 m 是输入降采样尺寸，1.0 m 是后续高斯体素尺寸，两者不能混同。[02 PCL 示例源码][s02]

可选择先以 VGICP 获取稳定初值、再用更细的 GICP 复核，但这是可选的多阶段方案。已有好初值时，应直接评估精配准配置是否足够，无需为了“流程完整”强加粗阶段。

## 4. 好初值下的标定：减少局部极值风险，但不消除模型误差

### 4.1 里程计、定位与外参标定的目标不同

里程计通常估计随时间变化的相对运动；定位估计当前帧到地图的位姿；刚性多传感器标定估计一组应跨观测保持一致的外参。它们都能调用局部配准，但不能因为单帧配准收敛就证明外参标定可靠。

好初值往往改善对应关系并降低落入错误局部解的风险，但它不是先验约束：普通 `align` 不会因为你确信初值就限制最终位姿在附近。算法仍在优化输入点云定义的目标。[helper 实现][helper][optimizer][optimizer]

固定平移或带置信度的外参先验属于额外问题建模，见[仅优化旋转的 GICP](../仅优化旋转的gicp/README.md)。平移不准却硬固定，可能造成旋转偏差；好的初值与可信的硬约束不是一回事。

### 4.2 示例确实存在速度、模型细节和停止精度的取舍

| 因素           | 源码事实                                   | 对标定的含义（工程判断）                               |
| -------------- | ------------------------------------------ | ------------------------------------------------------ |
| 降采样         | 示例使用 0.25 m                            | 可能不足以保留小尺度结构，但 0.25 m 不是位姿精度下限   |
| 协方差邻域     | 01 默认使用 10 邻点；02 使用 20            | 法向/局部平面模型不同，误差差异不能归因于接口          |
| 默认停止条件   | helper 为旋转 0.1°、平移 1 mm 的增量阈值  | 仅是“本轮更新足够小”，不是绝对外参误差保证           |
| PCL 封装默认值 | 旋转 0.002 rad、平移 0.0005 m              | 两路径原样运行没有完全对齐设置                         |
| 协方差正则化   | 保留局部特征向量，特征值设为`(1e-3,1,1)` | 强调平面法向约束，不是传感器标定过的绝对噪声协方差     |
| 优化精度       | 内部 double；PCL 最终常用矩阵为 float      | 精度需求高时保留内部 double 结果，也要检查输入坐标表示 |

来源：[helper 配置][setting]、[停止条件][criteria]、[PCL 实现][pclimpl]、[协方差估计][normal]。停止条件头文件的成员注释存在单位文字对调，应按构造值与 `converged()` 实际索引理解。

由于默认协方差是几何正则化模型，不能未经噪声模型校验就把 $H^{-1}$ 当成具有绝对物理置信度的外参协方差。[协方差估计][normal]

GICP 的法向约束和平面协方差建模也可参照作者的 [Generalized-ICP 论文](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf)。本文具体参数以当前源码为准。

### 4.3 面向标定的建议流程

以下是实验设计建议，尚未在你的标定数据上验证：

1. **先确认坐标与时间**：明确 $T_{target\leftarrow source}$、米/弧度单位、时间同步、运动补偿和静态重叠区域。算法可能用外参变化去解释时间误差或扫描畸变。
2. **用较好初值直接建立精配准基线**：先固定算法与数据子集，评估 GICP；大数据再比较 VGICP。对应阈值应覆盖初值不确定性、量测误差和离散化误差，并非越小越准。小角度下旋转误差带来的点位移约为 $\|p\|\delta\theta$，远处点尤其敏感。
3. **分开改变各项参数**：降采样尺寸、协方差邻域、对应阈值、停止阈值一次改变一类；若需要逐层精化，重新生成匹配该层点云的预处理缓存。
4. **适当收紧停止阈值并增加迭代预算**：同时观察代价、有效对应和实际位姿变化。极小增量可能来自退化、阻尼或错误对应，不等于高精度。
5. **验证几何可观测性**：查看未加先验/阻尼的 Hessian，检查场景是否仅一个平面、窄视场或单一方向。不要把人为大对角先验造成的条件数改善当成数据自身信息增加。
6. **跨场景验证**：用多个独立静态观测检查同一外参的一致性，并保留未参与调参的数据。需要联合估计时，应让多个观测共享一个外参变量；一次 `align` 不会自动完成多帧标定。

好初值可以节省寻找收敛域的工作，但不能补偿时间不同步、非刚性目标、系统测距偏差、错误对应和不可观测方向。更密点云和更严格阈值也不能保证真实误差单调改善。

## 5. 如何解释我们已经运行过的结果

此前官方示例在同一组 PLY 数据上的输出，与仓库 `T_target_source.txt` 的平移差约为：基础 GICP 6.87 mm、PCL 点云 GICP 8.84 mm、PCL 封装 VGICP 首次配准 17.68 mm。这些值来自先前运行及打印精度，本次没有重跑。

这些数值只能作为该次示例运行的参考对照，不能据此给出“PCL 更差”或“VGICP 必然更不准”的结论：算法、邻域和停止条件没有统一，也未做重复试验、旋转误差完整评估或耗时基准。文件中的参考矩阵也未在本次调研中确认其测量来源与不确定度，因此这里不将它当成具有已知精度的标定真值。

## 6. 源码索引

下面的链接对应本地核对的提交。遇到不同版本，应重新检查默认值、体素搜索策略及缓存失效条件。
