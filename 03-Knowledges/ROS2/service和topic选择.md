---
title: ROS 2 中 service 与 topic 的选择和接口设计
date: 2026-10-14
tags: [ROS2, service, topic, action, QoS, 接口设计, 移动机器人]
---
# ROS 2 中 service 与 topic 的选择和接口设计

在移动机器人算法开发中，`topic`、`service` 和 `action` 不是三种“传输速度不同”的接口，而是三种不同的交互语义：

- **topic**：持续产生的数据流，发布者和订阅者解耦；
- **service**：一次请求对应一次响应，适合短时、可明确结束的操作；
- **action**：可持续一段时间的目标，支持反馈、取消和最终结果。

最先问的问题不是“这个数据量大不大”，而是：**这是持续存在的事实、一次请求，还是一个需要过程控制的任务？**

## 1. 移动机器人里什么时候使用 topic

### 1.1 发布持续存在的状态或观测

只要数据代表机器人“现在是什么状态”或“刚刚观测到什么”，通常使用 topic：

- 激光雷达、深度图、相机图像、IMU、里程计；
- 当前位姿、速度、电池状态、定位质量；
- 局部代价地图、全局代价地图、规划路径；
- 底盘当前状态、故障状态、诊断信息。

这些数据的共同点是：发布者不需要知道当前是否有订阅者，订阅者也不应要求发布者为自己单独计算一次。新订阅者只关心后续数据；传感器数据通常还应有时间戳和坐标系信息。

例如：

```text
/sensors/imu              sensor_msgs/msg/Imu
/localization/odometry    nav_msgs/msg/Odometry
/perception/obstacles     vision_msgs/msg/Detection2DArray
/planning/local_path      nav_msgs/msg/Path
```

### 1.2 topic 不只是“高频数据”

低频状态也适合 topic。电池状态即使每秒只更新一次，仍然是“持续状态”；故障状态也可以以事件变化时发布，或按较低频率周期发布。频率不是唯一判断条件，**数据的生命周期和消费方式**更重要。

适合 topic 的典型特征：

- 可能有多个订阅者；
- 发布者不需要等待某个订阅者回复；
- 丢失一条旧消息通常可以接受，后面会有新消息；
- 订阅者加入后只需接收后续数据；
- 数据本身代表状态、观测或事件通知。

### 1.3 给 topic 选择 QoS

ROS 2 的 topic 通信还要定义 QoS。移动机器人中常见的思路是：

| 数据 | 常见 QoS 思路 | 原因 |
|---|---|---|
| 传感器实时流 | `best_effort`、小队列、`volatile` | 优先低延迟；旧帧过期后没有价值 |
| 关键状态或配置 | `reliable`，按需 `transient_local` | 希望尽量送达，后来加入的订阅者可能需要最近值 |
| 地图、静态标定 | `reliable`、`transient_local` | 新订阅者应能拿到已发布的最新静态数据 |
| 控制指令流 | 结合底盘接口和安全策略 | 要明确超时、停止和丢包后的安全行为 |

这只是设计起点，不能仅凭消息名称套用 QoS。发布者和订阅者的 QoS 必须兼容；还要考虑网络带宽、可靠性、延迟、历史深度和数据是否允许过期。ROS 2 的底层中间件提供 QoS 选项，具体行为和 RMW 实现也应在目标平台验证。[ROS 2 中间件与 QoS 概念](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html)

## 2. 什么时候使用 service

service 是一次请求和一次响应：客户端发出请求，服务端处理后返回结果。它适合**短时、边界清晰、结果可以一次返回**的操作：

- 查询当前地图、参数或设备状态；
- 设置一个参数或切换工作模式；
- 清除代价地图、重置里程计或重置故障；
- 加载/保存地图；
- 请求一次局部计算且能很快完成；
- 请求检测节点重新初始化或重新标定。

例如 `std_srvs/srv/SetBool` 适合“打开/关闭某个功能”，`nav_msgs/srv/GetPlan` 适合“给定起点和终点，返回一条规划结果”。ROS 2 的导航接口也把 `GetMap`、`GetPlan`、`LoadMap` 和 `SetMap` 定义为 service，体现了查询或一次性配置的语义。[ROS 2 `nav_msgs` 接口](https://docs.ros.org/en/ros2_packages/humble/api/nav_msgs/index.html)

### 2.1 service 的边界

不要把 service 当作“带返回值的话题”，也不要用它承载长时间任务：

- “开始建图并持续十分钟”不是好 service；
- “导航到目标点并反馈距离、支持取消”应使用 action；
- “持续发布当前地图或定位结果”应使用 topic；
- “启动一个算法线程后立即返回”要明确这只是启动命令，过程状态应由 topic 或 action 反馈。

service 调用仍可能超时、找不到服务端、服务端异常退出或返回业务失败。客户端不能只判断调用是否完成，还要检查响应中的业务状态。服务端也不应在回调中阻塞很久，否则会占用执行器线程，影响同节点的其他回调。

## 3. 什么时候使用 action

移动机器人中，导航、路径跟踪、机械臂运动、长时间录包或地图构建通常需要：

1. 发送目标；
2. 得到是否接受的确认；
3. 周期性收到反馈；
4. 能取消或抢占；
5. 最后得到成功、失败或取消结果。

这种语义应使用 action，而不是把“开始”和“查询进度”拆成一组 service。ROS 2 action 本身由 goal、feedback、result 组成，适合可抢占的长任务。

一个简单判断表：

| 问题 | 选择 |
|---|---|
| “现在的传感器/状态是什么？” | topic |
| “请执行一个很快完成的请求，并给我一次结果” | service |
| “请执行一个可能持续的任务，并让我看进度、取消或抢占” | action |
| “我只想通知某个事件，不要求对方回复” | topic（事件消息） |

## 4. 设计 service 之前先写行为契约

设计接口前不要先从字段名开始。先用一句话定义操作：

> 客户端在什么条件下请求什么动作，服务端在什么条件下接受、拒绝或执行失败，客户端如何知道最终状态？

以移动机器人“切换定位模式”为例，先写清楚：

- 调用者是谁，是否允许多个客户端调用；
- 当前模式是什么，目标模式有哪些；
- 切换需要多长时间，是否可能尚未完成就返回；
- 传感器或地图不满足条件时如何失败；
- 重复请求是否幂等；
- 运行过程中能否取消；
- 成功后哪些 topic 或状态会改变；
- 版本升级后旧客户端能否继续调用。

如果“切换”可能等待传感器、重定位或硬件响应很久，就应把它改成 action，或让 service 只负责提交请求并返回任务 ID，同时另有明确的状态反馈接口。不要用一个没有超时语义的 service 假装同步完成。

## 5. 如何定义 service 协议

ROS 2 `.srv` 由请求和响应两部分组成，中间用单独的 `---` 分隔。一个适合入门的模式是：请求表达**意图和条件**，响应表达**是否接受、业务状态、原因和必要结果**。

例如：

```srv
# srv/SetLocalizationMode.srv
string target_mode          # "map" / "odom" / "standby"
bool force_reinitialize
builtin_interfaces/Duration timeout
---
bool accepted
uint8 STATUS_OK=0
uint8 STATUS_INVALID_REQUEST=1
uint8 STATUS_BUSY=2
uint8 STATUS_PRECONDITION_FAILED=3
uint8 STATUS_INTERNAL_ERROR=4
uint8 status
string message
string active_mode
```

这个例子只是协议草案，真正落地前还要决定是否允许空字符串、`timeout` 为零代表什么，以及 `message` 是否仅用于人读。设计时注意以下原则：

### 请求字段

- **表达意图**：使用 `target_mode`、`map_name`、`reset_scope`，不要只放一个含义不清的 `data`；
- **约束输入**：能枚举的模式使用常量或明确的字符串集合；数值写单位、范围和坐标系；
- **区分可选项**：布尔值要能说明默认行为，必要时使用枚举而不是多个互相矛盾的布尔值；
- **考虑幂等性**：重复发送同一请求，是返回当前状态、重新执行，还是拒绝？要在接口说明里写明；
- **不要把大数据塞进请求**：图像、点云、地图等大数据应通过 topic/文件/资源引用传输，service 只传参数或 ID。

### 响应字段

- **区分传输成功与业务成功**：服务回调返回不代表动作成功，响应应有明确的 `status`；
- **给出机器可判断的状态码**：客户端根据枚举分支，不能解析中文或英文 `message`；
- **保留人类可读原因**：`message` 用于日志和调试，不作为程序逻辑条件；
- **返回必要上下文**：例如当前实际模式、使用的地图版本、生成的资源 ID；
- **失败也要定义内容**：失败时哪些字段有效，哪些字段为空或保持旧值。

### service 名称与语义

名称用动词或清楚的查询语义：`/reset_localization`、`/set_operating_mode`、`/get_map_metadata`。避免把 topic 名称加上 `/request` 就当成 service，也避免用一个 service 混合“查询、设置、启动任务”三种行为。

## 6. 如何设计配套的 message 协议

service 不是孤立的。一次操作通常还需要一个状态 topic、结果消息或 action。先区分三类消息：

### 6.1 状态消息：描述当前事实

```msg
# msg/LocalizationState.msg
std_msgs/Header header
uint8 MODE_STANDBY=0
uint8 MODE_ODOM=1
uint8 MODE_MAP=2
uint8 mode
uint8 health
string map_id
float32 covariance_score
string detail
```

状态 topic 应该能让一个刚订阅的节点理解“当前是什么状态”，而不是只发布发生过的事件。状态字段之间的单位、坐标系、时间含义和有效范围要写在接口注释或包文档中。

### 6.2 事件消息：描述发生过什么

例如 `LocalizationResetEvent.msg` 可以包含 `header`、`request_id`、`reason`、`success` 和 `status`。事件是否需要可靠 QoS、是否需要保存最近一条，要根据“漏掉一次是否会造成错误”决定；不能因为它叫 event 就默认使用同一种 QoS。

### 6.3 结果消息：描述一次请求的结果

如果服务只是短时操作，响应中直接返回结果即可；如果任务异步执行，结果应有 `request_id` 或 `goal_id`，并通过状态 topic 或 action result 让客户端关联。不要让客户端靠“下一条状态消息”猜它是不是自己的结果。

## 7. 一个移动机器人接口设计例子

需求：用户请求机器人清除局部代价地图。

### 方案 A：service

若清除动作在几十毫秒内完成，且不需要进度和取消，可以定义：

```srv
# srv/ClearLocalCostmap.srv
uint8 SCOPE_LOCAL=0
uint8 SCOPE_ALL=1
uint8 scope
string request_id
---
bool success
uint8 STATUS_OK=0
uint8 STATUS_INVALID_SCOPE=1
uint8 STATUS_NOT_READY=2
uint8 STATUS_INTERNAL_ERROR=3
uint8 status
string message
```

`request_id` 在需要审计、去重或与日志关联时有用；如果系统只允许一个客户端且调用本身可追踪，也可以不加。

### 方案 B：action

若清理包括停止传感器、重建地图或需要等待多个模块确认，则改为 action：

```text
Goal：scope、request_id
Feedback：stage、progress、detail
Result：success、status、message
```

中间过程的 `stage` 和 `progress` 属于 feedback，不应该通过高频 service 轮询模拟。

### 方案 C：topic

无论使用 service 还是 action，都可以保留：

```text
/localization/state
/localization/events
```

分别发布当前状态和已发生事件。这样监控、录包和 UI 不需要成为 service 的调用者，也不会因为没有主动调用而看不到系统状态。

## 8. 接口评审清单

在写 `.msg` 或 `.srv` 前逐项回答：

- 这是状态流、一次请求，还是长任务？为什么不是另外两种接口？
- 是否需要 action 的反馈、取消、抢占或最终结果？
- 谁是唯一的服务端？多个客户端同时请求时怎么办？
- 请求是否有超时、重试、去重和幂等语义？
- 传输成功与业务成功是否区分？失败是否有机器可判断的状态码？
- 每个数值的单位、坐标系、时间基准、有效范围是什么？
- 大数据是否错误地塞进 service？是否应该传 ID、引用或使用 topic？
- topic 的 QoS、队列深度、可靠性、持久性和过期策略是否与数据价值匹配？
- 接口是否能让新节点仅凭消息理解当前事实，而不依赖隐藏的调用顺序？
- 未来增加字段时是否保持兼容，是否需要接口版本或迁移策略？
- 是否已经写出最小调用示例、失败示例和状态变化？

## 9. 一句话决策

**持续变化的事实用 topic；短时、一次一答的操作用 service；可反馈、可取消的过程用 action。**先定义行为契约，再定义请求、响应和状态消息，最后才决定具体字段与 QoS。

参考：

- [ROS 2 官方概念：不同通信中间件与 QoS](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html)
- [ROS 2 `nav_msgs` 中的 service/message 接口](https://docs.ros.org/en/ros2_packages/humble/api/nav_msgs/index.html)
