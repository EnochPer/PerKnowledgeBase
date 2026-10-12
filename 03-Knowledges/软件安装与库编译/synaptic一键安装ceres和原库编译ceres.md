---
title: Ubuntu 22.04 中通过 Synaptic 安装与源码编译 Ceres
date: 2026-10-08
tags: [Ubuntu, ROS2, Ceres, Synaptic, CMake]
---
# Ubuntu 22.04 中通过 Synaptic 安装与源码编译 Ceres

## 1. 先确认当前版本

在 Ubuntu 22.04（Jammy）的官方 `universe` 仓库中，`libceres-dev` 和 `libceres2` 的版本是 `2.0.0+dfsg1-5`。**Synaptic 只是 APT 的图形界面**，因此刷新软件源后仍然只能看到这个版本；ROS 2 Humble 也不会自动把 Ceres 升级到 2.1 或 2.2。

```bash
apt-cache policy libceres-dev libceres2
```

输出中同时看到下面两项，说明编译头文件和运行库都来自同一个 APT 版本：

```text
Installed: 2.0.0+dfsg1-5
Candidate: 2.0.0+dfsg1-5
```

## 2. Ceres 2.0、2.1、2.2 的关系

- Ceres 2.0 只有旧的 `LocalParameterization` 接口，没有 `ceres::Manifold`。
- Ceres 2.1 是过渡版本，新增 `Manifold`，同时仍兼容 `LocalParameterization`。如果既要使用流形，又希望降低对旧 ROS/第三方代码的影响，优先选 2.1.0。
- Ceres 2.2 移除了 `LocalParameterization`，并要求 C++17 和较新的 CMake。旧代码若仍调用 `SetParameterization()`，通常需要改成 `SetManifold()`，不适合作为第一次升级目标。

因此，Ubuntu 22.04 上没有一个可靠的 Synaptic“升级按钮”可以完成这次升级。最简单且可回退的做法是：保留 APT 的 Ceres 2.0，在用户目录并行安装 Ceres 2.1.0，然后只让需要流形 API 的项目使用它。

## 3. 是否必须升级才能运行当前示例

不必须。`03-Knowledges/标定/带约束的位姿优化/只优化旋转rpy.md` 配套的 `demo.cpp` 使用的是三维 `delta` 参数块，并在残差中手动计算
`Exp(delta^) R0`。这是“手写局部增量”，不是 Ceres 2.1 才提供的 `ceres::Manifold`，所以 Ceres 2.0 可以编译这类实现。

只有在代码需要以下形式时，才需要至少升级到 Ceres 2.1：

```cpp
problem.SetManifold(parameter_block, new ceres::QuaternionManifold);
```

如果只是想避免 RPY 的万向锁，保留当前三维旋转增量方案即可；升级 Ceres 不是必要条件。

## 4. 推荐方案：并行源码安装 Ceres 2.1.0

这会编译 **Ceres 库源码**，不是编译 ROS 2，也不是要求把自己的项目改成 Ceres 源码仓库。建议安装到用户目录，避免覆盖 `/usr/lib` 中供 ROS 或其他系统包使用的 2.0。

### 4.1 安装编译依赖

下面命令只安装构建依赖，不会删除 APT 的 Ceres 2.0：

```bash
sudo apt update
sudo apt install -y build-essential cmake libgoogle-glog-dev libgflags-dev \
  libeigen3-dev libatlas-base-dev
```

大规模稀疏问题需要 SuiteSparse 时再安装可选依赖：

```bash
sudo apt install -y libsuitesparse-dev
```

### 4.2 下载、编译并安装到用户目录

```bash
mkdir -p "$HOME/src" "$HOME/.local/ceres-2.1.0"
cd "$HOME/src"
wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz
tar -xzf 2.1.0.tar.gz

cmake -S ceres-solver-2.1.0 -B ceres-build-2.1.0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/ceres-2.1.0" \
  -DBUILD_TESTING=OFF
cmake --build ceres-build-2.1.0 -j"$(nproc)"
cmake --install ceres-build-2.1.0
```

如果不需要 SuiteSparse，可以显式关闭它，减少依赖和配置问题：

```bash
cmake -S ceres-solver-2.1.0 -B ceres-build-2.1.0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/ceres-2.1.0" \
  -DBUILD_TESTING=OFF \
  -DSUITESPARSE=OFF
```

上面的 `cmake --build` 和 `cmake --install` 是实际编译步骤；本文只记录方法，没有执行它们。

### 4.3 让单个 CMake 项目使用 2.1.0

在项目配置时显式指定 Ceres 的 CMake 配置目录：

```bash
cmake -S . -B build-ceres21 \
  -DCeres_DIR="$HOME/.local/ceres-2.1.0/lib/cmake/Ceres"
cmake --build build-ceres21 -j"$(nproc)"
```

也可以在 `CMakeLists.txt` 中使用：

```cmake
find_package(Ceres 2.1 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE Ceres::ceres)
```

如果程序运行时找不到用户目录中的共享库，优先在项目中配置 RPATH；临时测试可使用：

```bash
export LD_LIBRARY_PATH="$HOME/.local/ceres-2.1.0/lib:$LD_LIBRARY_PATH"
```

不要把 2.1 的头文件和 2.0 的库混用，也不要把新库文件直接复制到 `/usr/lib`。

### 4.4 验证安装的版本

```bash
grep -E 'CERES_VERSION_(MAJOR|MINOR|REVISION)' \
  "$HOME/.local/ceres-2.1.0/include/ceres/version.h"
```

项目配置时还可以查看 CMake 实际找到的路径：

```bash
cmake -S . -B build-ceres21 \
  -DCeres_DIR="$HOME/.local/ceres-2.1.0/lib/cmake/Ceres" \
  -LAH | grep -E 'Ceres_DIR|CERES'
```

## 5. 什么时候考虑 Ceres 2.2

只有在项目及其依赖已经准备好 C++17，并且确认没有旧的 `LocalParameterization` 调用时，再考虑 2.2。升级 2.2 通常需要：

1. 将 `SetParameterization()` 和自定义 `LocalParameterization` 改为 `Manifold` API；
2. 检查 CMake、Eigen、SuiteSparse 等依赖版本；
3. 对所有使用 Ceres 的 ROS 包分别重新配置和验证。

对于 ROS 2 Humble，2.1.0 的并行安装通常比把系统 Ceres 替换成 2.2 更稳妥。除非确实需要 2.2 的新功能，否则没有必要承担这次 API 迁移。

## 6. 结论

- 仅使用当前 `demo.cpp`：不需要升级，Ceres 2.0 已足够。
- 需要 `ceres::Manifold`：源码安装 Ceres 2.1.0，建议并行安装到 `$HOME/.local/ceres-2.1.0`。
- 不建议通过混用其他 Ubuntu 版本的软件源、随意 PPA 或直接覆盖 `/usr` 来“升级” Ceres。
- 这里需要编译的是 Ceres 库源码；自己的项目只需通过 `Ceres_DIR` 选择新版本，不需要把项目放进 Ceres 源码仓库。

## 7. 参考

- [Ubuntu Jammy 的 libceres-dev 包信息](https://packages.ubuntu.com/en/jammy/libceres-dev)：版本为 `2.0.0+dfsg1-5`。
- [Ceres 版本历史](https://github.com/ceres-solver/ceres-solver/blob/master/docs/source/version_history.rst)：2.1 引入 `Manifold` 过渡 API，2.2 移除 `LocalParameterization`。
- [Ceres 安装说明](https://ceres-solver.org/installation.html)：Ubuntu 22.04 的源码构建和依赖说明。
