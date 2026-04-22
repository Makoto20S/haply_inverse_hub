# Haply Inverse3 遥操作项目 —— 面试高频 20 题

> 覆盖项目理解、架构数据流、ROS2、WebSocket、控制算法、工程实践六大维度

---

## 🔹 一、项目理解类

### Q1：用一句话概括这个项目是做什么的？

**答**：基于 Haply Inverse3 力反馈主手，通过 ROS2 跨机通信实现对 Franka FR3 机械臂的双向力反馈遥操作——主手控制从手运动，从手把末端接触力实时反馈回主手，让操作者获得"临场感"。

---

### Q2：这个项目为什么是"上位机"代码？下位机做什么？

**答**：

本仓库是**主手端（上位机）**，部署在操作者的 Ubuntu PC 上，负责：
- 读取 Inverse3 手柄的位置/姿态/按钮
- 坐标映射（设备系 → 机器人基系）
- 力反馈计算（阻抗+传感器力混合）
- ROS2 话题收发

**下位机（Franka 工控机）**负责：
- 订阅 `/device/pose` 目标位姿，驱动 Franka FR3 运动
- 读取末端力传感器数据
- 把力数据和实际位置发布到 `/end_effector_force`

---

### Q3：系统中有哪些关键坐标系？它们之间怎么映射？

**答**：有两个核心坐标系：

- **设备坐标系**（Device）：手柄自身的 XYZ
- **机器人基坐标系**（Robot Base）：Franka 的 base 系

映射关系是一个 **绕 Z 轴旋转 90°** 的变换矩阵：

```
R_DEV2ROB = [[0, -1, 0],
             [1,  0, 0],
             [0,  0, 1]]
```

即：
- 设备 X → 机器人 -Y
- 设备 Y → 机器人 X
- 设备 Z → 机器人 Z（不变）

**位置映射公式**：
```
delta_device = dev_pos - device_origin          # 手柄相对锚点的位移
scaled = delta_device * position_scale           # 缩放
delta_robot = R_DEV2ROB @ scaled                # 旋转到机器人坐标系
target_pos = robot_base_pos + delta_robot       # 叠加到机器人基准位置
```

---

## 🔹 二、架构与数据流类

### Q4：画一下这个系统的数据流图，从手柄到机械臂再回传到手柄。

**答**：

```
┌─────────────────────────────────────────────────────────────┐
│                      上位机 (Ubuntu PC)                      │
│                                                              │
│  Inverse3手柄 ──WebSocket(~1kHz)──► inverse_comm.py        │
│                                          │                  │
│                                   device_wrapper.py          │
│                                          │                  │
│                              device_node.py (100Hz定时器)    │
│                                   /            \             │
│                         Publish:               Subscribe:    │
│                    /device/pose            /end_effector_force│
│                    /gripper_command        /pose_echo         │
│                    /device/force_z              ▲             │
│                         ▼                       │             │
│                    ROS2 DDS (WiFi) ◄────────────┘             │
│                         ▼                                    │
│                  Franka FR3 工控机                            │
│                         ▼                                    │
│                  控制器 → 机械臂运动                           │
│                         ▼                                    │
│                  力传感器 → 回传力+位置                        │
└─────────────────────────────────────────────────────────────┘
```

| 方向 | 话题 | 频率 | 内容 |
|------|------|------|------|
| 主→从 | `/device/pose` | 100 Hz | 目标位姿 |
| 主→从 | `/gripper_command` | 2 Hz | 夹爪指令 |
| 主→从 | `/device/force_z` | 20 Hz | Z轴期望力 |
| 从→主 | `/end_effector_force` | ~100 Hz | 传感器力+实际位置 |
| 从→主 | `/pose_echo` | 100 Hz | RTT回弹测延迟 |

---

### Q5：为什么手柄通信要单独起一个线程？不能和 ROS2 节点在同一个线程吗？

**答**：手柄数据频率约 **1kHz**，ROS2 节点主循环是 **100Hz**，两者差一个数量级。

如果放在同一个线程：
- WebSocket `recv()` 是阻塞等待的，会卡住 ROS2 的 100Hz 定时器
- 或者反过来，ROS2 的回调执行会耽误 WebSocket 及时收数据

**解决方案**：`inverse_comm.py` 在后台守护线程中运行 `asyncio` 事件循环，通过 `threading.Lock` 保护的共享变量与 ROS2 主线程交换数据，实现解耦。

---

### Q6：遥操作的"使能/禁用"逻辑是怎么设计的？为什么松开 B 键不会导致位姿跳变？

**答**：**B 键**是遥操作使能开关：

| 动作 | 行为 |
|------|------|
| **按住 B** | 记录当前手柄位置为 `device_origin`，记录当前机器人目标位置为 `robot_base_pos`，开始映射 |
| **松开 B** | 把当前平滑后的目标位置 `smoothed_pos` 保存为新的 `robot_base_pos`，禁用遥操作，机器人停在当前位置 |
| **再次按住 B** | 新的 `device_origin` 锚定，映射基于新的 `robot_base_pos` 继续 |

**不跳变的关键**：松手时把"当前目标位姿"冻结为下次的基准点，而不是回到零点。这样每次重新使能都是从冻结位置无缝继续。

---

### Q7：项目中用了几种线程/进程模型？各自做什么？

**答**：

| 执行单元 | 来源 | 职责 |
|---------|------|------|
| ROS2 主线程 | `rclpy.spin(node)` | 运行 Timer 回调（100Hz）、Subscription 回调 |
| WebSocket 通信线程 | `threading.Thread(target=device.run_inverse)` | 运行 `asyncio` 事件循环，与 Haply SDK 收发 JSON（~1kHz） |
| 隐含的 DDS 线程 | ROS2 底层 (rmw_cyclonedds_cpp) | 处理网络收发、话题发现、节点发现 |

**共享状态保护**：`position`、`buttons`、`_external_force` 等通过 `threading.Lock` 保护，防止竞态条件。

---

## 🔹 三、ROS2 知识类

### Q8：ROS2 的 Topic 和 Service 有什么区别？这个项目为什么用 Topic 不用 Service？

**答**：

| 特性 | Topic | Service |
|------|-------|---------|
| 通信模式 | 发布-订阅，多对多 | 客户端-服务器，一对一 |
| 实时性 | 连续流，高频，非阻塞 | 请求-响应，低频，阻塞等待 |
| 典型用途 | 传感器数据、控制指令流 | 配置查询、开关动作、动作请求 |
| 是否保证送达 | 取决于 QoS | 保证请求-响应配对 |

**本项目用 Topic 的原因**：
1. 位姿、力反馈都是**高频连续流数据**（100Hz），Topic 的发布-订阅模型天然适合
2. Service 是阻塞的，100Hz 下频繁调用 Service 会导致严重延迟
3. 上位机和下位机是**松耦合**的，不需要知道对方是否存在也能发布/订阅

---

### Q9：QoS 是什么？这个项目怎么配置的？BEST_EFFORT 和 RELIABLE 有什么区别？

**答**：QoS（Quality of Service）是 ROS2 控制消息传输策略的机制，通过配置 `depth`、`reliability`、`history` 等参数，在不同场景下权衡实时性和可靠性。

**本项目的 QoS 配置**：

| 话题 | 策略 | depth | 原因 |
|------|------|-------|------|
| `/device/pose` | BEST_EFFORT | 10 | 位姿流，最新帧最重要 |
| `/end_effector_force` | BEST_EFFORT | 10 | 力反馈流，实时性优先 |
| `/gripper_command` | BEST_EFFORT | 10 | 夹爪指令，但有 2Hz 心跳兜底 |

**BEST_EFFORT vs RELIABLE**：

| | BEST_EFFORT | RELIABLE |
|--|-------------|----------|
| 传输语义 | 类似 UDP，尽力发送，丢了不补 | 类似 TCP，丢了重传，保证送达 |
| 延迟 | 低 | 稍高（重传开销） |
| 适用场景 | 高频实时数据，最新值更重要 | 必须送达的指令或配置 |

**坑点**：发布端和订阅端的 QoS 策略必须兼容。如果发布端是 BEST_EFFORT，订阅端是 RELIABLE，两者可能无法匹配，导致静默收不到消息。调试时应使用 `ros2 topic info` 检查双方的 QoS。

---

### Q10：`Wrench` 和 `WrenchStamped` 有什么区别？为什么从端用 `Wrench`？

**答**：

| | `Wrench` | `WrenchStamped` |
|--|---------|-----------------|
| 包含字段 | `force(x,y,z)` + `torque(x,y,z)` | `header` + `wrench` |
| 时间戳 | 无 | 有（`header.stamp`） |
| 坐标系 | 无 | 有（`header.frame_id`） |

从端发布的是 `Wrench`，所以订阅端也必须用 `Wrench` 订阅。

**常见调试陷阱**：如果订阅端用了 `WrenchStamped` 而发布端是 `Wrench`，ROS2 **不会报错**，但回调函数永远不会触发，表现为"收不到消息"。排查时应先用 `ros2 topic info` 确认双方的消息类型一致。

---

### Q11：ROS2 参数系统（Parameter）在这个项目中怎么用的？有什么好处？

**答**：`device_node.py` 的 `__init__()` 中用 `declare_parameter()` 声明了 30+ 个参数，涵盖坐标映射、力反馈、阻抗控制、接触检测等所有可调节的行为参数。

**代码示例**：
```python
# 声明参数（带默认值）
self.declare_parameter('position_scale', 1.0)
self.declare_parameter('impedance_k_z', 100.0)

# 读取参数
scale = self.get_parameter('position_scale').value
```

**使用方式**：
```bash
# 命令行覆盖
ros2 run teleop_nodes device_node --ros-args -p position_scale:=1.5

# 运行时动态修改
ros2 param set /device_node impedance_k_z 120.0

# YAML 文件批量加载
ros2 launch teleop_nodes teleop_launch.py params_file:=/path/to/params.yaml
```

**好处**：
- **运行时可调**：不用改代码、不用重启节点
- **配置集中**：所有参数统一管理，便于实验调参
- **版本控制友好**：YAML 参数文件可以入库，代码不用变

---

## 🔹 四、WebSocket / 通信类

### Q12：WebSocket 和 TCP Socket 有什么区别？为什么不用原始 TCP？

**答**：

| 特性 | TCP Socket | WebSocket |
|------|-----------|-----------|
| 协议层次 | 传输层 | 应用层（基于 TCP） |
| 数据单位 | 裸字节流 | 帧（文本/二进制，边界清晰） |
| 通信模式 | 双全工，但需自己设计协议 | 天然双全工，帧协议内置 |
| 握手 | 直接连接 | HTTP 兼容握手，易穿透防火墙/Nginx |
| 适用场景 | 自定义二进制协议 | 实时推送、聊天、硬件通信 |

**本项目用 WebSocket 的原因**：
- Haply Desktop SDK 暴露的就是 WebSocket 服务端（`ws://localhost:10001`）
- JSON 消息边界清晰，无需处理 TCP 粘包/拆包
- 与 HTTP 兼容的握手便于调试（可用浏览器 DevTools 直接连）

---

### Q13：`orjson` 是什么？为什么不用 Python 内置的 `json` 模块？

**答**：`orjson` 是一个用 **Rust** 编写的高性能 JSON 库。

手柄数据频率约 **1000Hz**，每毫秒都要解析和序列化 JSON。在实时通信场景下：
- `orjson` 比标准库 `json` **快几十倍**
- 能显著降低 CPU 占用和通信延迟
- API 兼容（`orjson.loads()` / `orjson.dumps()`），替换成本低

---

## 🔹 五、控制算法与力反馈类

### Q14：力反馈的处理管线是怎样的？从收到从机消息到主手感受到力，经历了哪些步骤？

**答**：完整处理管线共 11 步：

```
① 提取 ──► 从 Wrench 解析 force(传感器力) + torque(实际位置)
    │
② 过滤 ──► 全零位置帧过滤（防止异常跳变）
    │
③ 速度估计 ──► 数值微分 + 低通滤波，得到实际末端速度
    │
④ 传感器力处理 ──► 按轴缩放 → 截断 → 低通滤波
    │
⑤ 阻抗计算（仅遥操作使能时）
    ├─ 位置误差 = target_pos - actual_pos
    ├─ 径向死区（误差<18mm时忽略）
    ├─ 弹簧力 = K_eff × 有效误差
    └─ 阻尼力 = C × d_error/dt
    │
⑥ 接触检测 ──► 多条件投票（误差大+误差增长快+传感器力阶跃）→ 触发触觉脉冲
    │
⑦ 变刚度 ──► 根据"跟踪比"动态调整 K
    │
⑧ 加权混合 ──► combined = -0.7×sensor_force + 0.3×impedance_force + pulse
    │
⑨ 安全限幅 ──► 合力硬上限 8N
    │
⑩ 坐标变换 ──► robot 系 → device 系
    │
⑪ 输出 ──► DeviceWrapper.set_external_force() → 手柄电机
```

---

### Q15：什么是阻抗控制？项目中怎么实现的？

**答**：阻抗控制是让机器人（或手柄）表现出类似**"弹簧-阻尼-质量"**的力学特性，而不是直接控制力或位置。

**核心公式**：
```
F = K × Δx + C × ẋ
```

| 参数 | 含义 | 物理直觉 |
|------|------|---------|
| K（刚度） | N/m | 误差越大，反馈力越强。像弹簧 |
| C（阻尼） | N·s/m | 抑制振荡，让手感"黏"而不是"弹" |

**项目实现细节**：
- 当主手目标位姿和从手实际位姿有偏差时，计算弹簧力和阻尼力
- **K 是可变的**：根据"跟踪比"（实际位移/目标位移）动态调整
  - 自由空间（跟踪比≈1）→ K 小，移动轻松
  - 硬接触（跟踪比≈0）→ K 大，阻力明显
- 最终混合到主手反馈力中，让操作者感受到"机器人没跟上"的阻力

---

### Q16：接触检测是怎么做的？触觉脉冲有什么作用？

**答**：采用**多条件投票状态机**：

| 条件 | 权重 | 含义 |
|------|------|------|
| A：位置误差 > 死区（18mm） | 1票 | 主从位置已经不同步 |
| B：误差增长率 > 阈值（0.04 m/s） | 1票 | 误差在快速扩大 |
| C：传感器力阶跃 > 阈值（2.5N） | 2票 | 从端确实碰到东西了 |

**状态转移**：
- `votes >= 3` 且冷却时间（0.5s）已过 → 从 `free` 转入 `contact`
- 误差回到死区一半以内 → 从 `contact` 转回 `free`

**触觉脉冲**：
触发瞬间给主手一个短时、方向明确的力脉冲：`F_pulse = amplitude × exp(-t/τ) × direction`

**作用**：让操作者在视觉延迟之前**先通过触觉感知到接触发生**，这是遥操作"临场感"的关键设计。视觉延迟可能 20~50ms，而触觉延迟只要几毫秒。

---

### Q17：从机为什么把位置数据放在 `Wrench.torque` 字段里传？

**答**：这是一种**字段借用（hack/trick）**。

`Wrench` 消息只有 6 个浮点数字段：`force(x,y,z)` + `torque(x,y,z)`。如果同时传"末端力"和"末端位置"，有两种选择：

| 方案 | 做法 | 问题 |
|------|------|------|
| A | 发两个话题 | 增加网络开销，且两帧数据可能不同步 |
| B | 发一个话题但借用字段 | 一个消息同时带力和位置，但需要双方约定 |

从端选择方案 B：`force` 的三个分量传三维力，`torque` 的三个分量传三维实际位置。订阅端解析时按约定拆分即可。这是工程中常见的"在有限消息类型上做扩展"的做法。

---

## 🔹 六、工程实践与优化类

### Q18：项目中做了哪些线程安全措施？

**答**：

1. **`threading.Lock` 保护共享状态**：`inverse_comm.py` 中所有可能被多线程访问的数据（`position`、`velocity`、`buttons`、`_external_force`）都通过 `with self._lock` 进行原子读写

2. **守护线程（Daemon Thread）**：WebSocket 通信线程设 `daemon=True`，当主程序退出时自动结束，不会导致进程挂死

3. **超时与掉线检测**：`device_wrapper.py` 中通过 `stale_threshold=0.5s` 判断数据是否过期，超过则 `is_connected=False`

4. **异常重连机制**：`inverse_comm.py` 的 `main()` 里有 `max_retries` 重试逻辑，连接断开时会自动重连

---

### Q19：如果力反馈回调处理太慢，会导致什么问题？怎么解决？

**答**：ROS2 默认使用 **SingleThreadedExecutor**，所有回调（Timer + Subscription）在同一个线程中**串行执行**。

**问题**：如果 `_ee_force_callback` 中的阻抗计算管线耗时太长（例如矩阵运算、大量滤波），会**阻塞 `_timer_callback`**，导致：
- `/device/pose` 发布延迟、抖动
- 机器人运动出现肉眼可见的卡顿
- 有效频率从 100Hz 掉到 60Hz 甚至更低

**解决方案**（已在代码注释中提及改进方向）：
1. 使用 `MultiThreadedExecutor` 替代默认执行器，提供多个回调线程
2. 把力回调放入 `ReentrantCallbackGroup`，让它和 Timer 回调**并行执行**
3. 简化力计算管线，降低单次回调耗时
4. 力计算中避免动态内存分配（预分配 numpy 数组）

---

### Q20：如果要你优化这个系统的实时性，你会从哪些方面入手？

**答**：可以从**计算并行化**、**通信优化**、**算法轻量化**、**运行时调优**四个层面入手：

| 层面 | 具体措施 |
|------|---------|
| **计算并行化** | 1. `MultiThreadedExecutor` + `ReentrantCallbackGroup`，让 Timer 和 Subscription 回调并行<br>2. 力计算管线中独立的 XYZ 三轴计算可尝试向量化或并行化 |
| **通信优化** | 3. 确认发布端和订阅端 QoS 策略完全匹配，避免静默丢消息<br>4. 有线以太网替代 WiFi，降低网络抖动<br>5. 优化 CycloneDDS XML 配置，调大缓冲区、调整发现机制 |
| **算法轻量化** | 6. 阻抗计算中部分滤波可降阶或换用更轻量的实现<br>7. 接触检测状态机可用查表法替代部分浮点运算<br>8. 预分配所有 numpy 数组，避免运行时 `malloc` |
| **运行时调优** | 9. 已用 `gc.disable()` 禁用垃圾回收，可进一步结合 `gc.freeze()`<br>10. ROS2 `loaned messages` 零拷贝机制减少序列化开销<br>11. 进程绑核（`taskset`）减少 CPU 调度抖动 |

---

## 附录：面试表达技巧

- **先讲架构，再讲细节**：被问到"这个项目做了什么"时，先用 30 秒画数据流图，再展开某个模块
- **主动暴露问题和解决过程**：比如"QoS 不匹配导致力话题静默收不到，我通过 `ros2 topic info` 发现类型不一致，定位花了 2 小时"
- **强调安全设计意识**：8N 硬限幅、零值过滤、径向死区、合力截断——这些体现工程思维
- **表现出对实时性的理解**：100Hz 意味着什么（10ms 周期），什么操作会阻塞它，怎么解
