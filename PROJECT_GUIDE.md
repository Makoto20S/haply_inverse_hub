# Haply Inverse3 遥操作主手端 —— 完全学习指南

> 适合对象：**零基础 ROS2**，想了解本项目每个代码文件的作用、数据流和涉及的所有 ROS2 知识点。

---

## 一、一句话概括这个项目

**用 Haply Inverse3 力反馈手柄（主手）控制 Franka FR3 机械臂（从手），实现双向力反馈遥操作。**

你手握主手移动 → 机械臂跟着动；机械臂碰到东西 → 你手上感受到力。

---

## 二、先搞懂 5 个核心概念（零基础必读）

| 概念 | 通俗解释 | 在本项目中的角色 |
|------|---------|---------------|
| **ROS2** | 机器人操作系统，让不同电脑上的程序能互相发消息通信 | 上位机（你的电脑）和下位机（Franka 工控机）通过 ROS2 交换数据 |
| **Node（节点）** | ROS2 中的一个程序进程，做一件事 | `device_node.py` 就是一个节点，负责读取手柄 + 发位姿 |
| **Topic（话题）** | 节点之间发消息的"频道"，发布-订阅模式 | `/device/pose` 是一个话题，主手发布位姿，从手机器人订阅 |
| **Message（消息）** | 话题上传递的数据包，有固定格式 | `PoseStamped` 是消息类型，包含位置和姿态 |
| **QoS** | 服务质量策略，决定消息传输的可靠性/实时性 | `BEST_EFFORT` 适合高频实时数据，`RELIABLE` 适合重要指令 |

---

## 三、系统架构总览（最重要的一张图）

```
┌─────────────────────────────────────────────────────────────────────┐
│                        上位机（你的 Ubuntu PC）                        │
│                                                                     │
│  ┌─────────────┐     WebSocket      ┌──────────────────────────┐   │
│  │ Haply Desktop│◄─────────────────►│   inverse_comm.py (Device)│   │
│  │  (SDK服务端) │   ws://localhost   │   收发JSON · ~1kHz循环     │   │
│  └─────────────┘   :10001           └──────────┬───────────────┘   │
│                                                │                    │
│                                         device_wrapper.py           │
│                                         四元数修正 · 按钮边沿检测      │
│                                                │                    │
│                                    ┌───────────────────┐           │
│                                    │  device_node.py    │           │
│                                    │  ROS2 DeviceNode   │           │
│                                    │  坐标映射 · 力反馈管线│           │
│                                    └────────┬──────────┘           │
│                                             │                       │
│           发布(Publish)                     │      订阅(Subscribe)  │
│              ▼                              │              ▲        │
│    ┌─────────────────┐              ROS2 DDS (WiFi)     ┌────────┐│
│    │ /device/pose    │─────────────────────────────────►│ 从机   ││
│    │ /gripper_command│─────────────────────────────────►│ Franka ││
│    │ /device/force_z │─────────────────────────────────►│ 工控机 ││
│    │ /device/state   │─────────────────────────────────►│        ││
│    └─────────────────┘                                └────────┘│
│                                              ▲                    │
│    ┌─────────────────┐                       │                    │
│    │ /end_effector_force│◄───────────────────┘                    │
│    │ /pose_echo         │◄──────────────────── RTT测量             │
│    └─────────────────┘                                            │
└─────────────────────────────────────────────────────────────────────┘
```

### 数据流方向总结

| 方向 | 话题名 | 消息类型 | 频率 | 内容 |
|------|--------|---------|------|------|
| 主→从 | `/device/pose` | `PoseStamped` | 100 Hz | 目标位姿（位置+姿态） |
| 主→从 | `/gripper_command` | `Int32` | 2 Hz | 夹爪开合（0=开, 1=合） |
| 主→从 | `/device/force_z` | `Float64` | 20 Hz | Z轴期望力（向下推的力） |
| 从→主 | `/end_effector_force` | `Wrench` | ~100 Hz | 末端力传感器数据+实际位置 |
| 从→主 | `/pose_echo` | `PoseStamped` | 100 Hz | RTT回弹（测网络延迟） |

---

## 四、每个代码文件是干嘛的？（逐文件解析）

### 4.1 硬件通信层

#### `Inverse_controller/inverse_comm.py` —— WebSocket 通信客户端

**作用**：通过 WebSocket 连接 Haply Desktop SDK，和 Inverse3 手柄+VerseGrip 控制器实时通信。

**核心流程**：
```
Haply Desktop (ws://localhost:10001)
    │
    ▼ 每 ~1ms 发一次 JSON
WebSocket 连接
    │
    ▼ orjson 解析
提取位置/速度/按钮/姿态
    │
    ▼ threading.Lock 保护
写入实例变量 (self.position, self.buttons...)
    │
    ▼ 同时反向发送力指令
把 calculate_total_force() 算出的力发回设备
```

**关键技术点**：
- `asyncio` + `websockets`：异步通信，不阻塞主线程
- `orjson`：高性能 JSON 解析（比普通 json 快几十倍）
- `threading.Lock`：线程锁，保护共享数据（ROS2 主线程和 WebSocket 线程同时读写）
- 在 `__main__` 里用 `threading.Thread` 起守护线程运行 `run_inverse()`

**为什么要单独线程？** 手柄数据 ~1kHz，ROS2 节点主循环 100Hz，不能互相阻塞。

---

#### `Inverse_controller/device_wrapper.py` —— 设备状态封装器

**作用**：把 `inverse_comm.py` 的原始数据包装成干净、好用的 `DeviceState` 数据结构。

**核心功能**：

| 功能 | 说明 |
|------|------|
| 四元数连续性修复 | 防止四元数符号翻转（q 和 -q 表示同一旋转）导致的姿态跳变 |
| 按钮边沿检测 | 检测"刚按下"(pressed) 和 "刚松开"(released)，不是"是否按住" |
| 连接状态判断 | 数据超过 0.5 秒没更新 → 判定掉线 |
| 格式转换 | xyzw（设备格式）↔ wxyz（MuJoCo/ROS 常用格式） |

**数据结构 `DeviceState`**：
```python
@dataclass
class DeviceState:
    timestamp: float          # 采样时间
    position: np.ndarray      # [x,y,z] 米
    velocity: np.ndarray      # [vx,vy,vz] 米/秒
    force: np.ndarray         # [fx,fy,fz] 牛顿
    orientation_xyzw: np.ndarray  # 设备原始格式 [x,y,z,w]
    orientation_wxyz: np.ndarray  # ROS常用格式 [w,x,y,z]
    buttons: List[bool]       # [A,B,C] 当前状态
    buttons_pressed: List[bool]   # 上升沿（本次刚按下）
    buttons_released: List[bool]  # 下降沿（本次刚松开）
    is_connected: bool
    data_rate_hz: float
```

**为什么要有 wrapper？** `inverse_comm.py` 只管原始通信，wrapper 负责数据清洗和高层语义，下游代码（device_node.py）只和 wrapper 打交道。

---

#### `Inverse_controller/force_controller_1.py` —— 力反馈计算器

**作用**：计算要发给 Inverse3 电机的力指令。

**当前状态**：虚拟墙/地板已禁用（返回零），实际只做了：
1. **外部力透传**：接收 `device_node.py` 算出的力，直接叠加
2. **低通滤波**：`filtered_force = alpha * new + (1-alpha) * old`
3. **力限幅**：合力超过阈值则按比例缩小
4. **快速归零**：力很小时强制清零，防止滤波拖尾导致"粘滞感"

**为什么禁用虚拟墙？** 因为遥操作场景下，力反馈主要来自从端机械臂的真实传感器力，不需要本地虚拟墙。

---

#### `Inverse_controller/transformation.py` —— 四元数工具

**作用**：两个纯数学函数。

| 函数 | 作用 |
|------|------|
| `fix_quat_continuity()` | 修复四元数符号翻转。如果当前帧和上一帧点积<0，说明跳了符号，取反修正 |
| `quat_to_axes_xyzw()` | [x,y,z,w] → [w,x,y,z] 格式转换 |

**四元数符号翻转是什么？** 四元数 q 和 -q 表示完全相同的旋转，但硬件可能在这两个等价表示之间跳来跳去，导致姿态数据突变。点积<0 就说明翻了，把它翻回来。

---

### 4.2 配置管理层

#### `Inverse_controller/config_manager.py` —— TOML 配置管理

**作用**：读取 `config/leader.toml` 或 `config/follower.toml`，提供 `get_value("network.uri")` 式的层级访问。

**核心设计**：
- `ConfigMode.LEADER` = 主手真机配置
- `ConfigMode.FOLLOWER` = 从端仿真配置
- 配置文件不存在时，自动创建默认配置
- 全局单例模式：整个项目共用同一个 `ConfigManager` 实例

---

### 4.3 ROS2 核心节点

#### `device_node.py` / `ros_ws/src/teleop_nodes/teleop_nodes/device_node.py` —— 主节点（~1100行，最核心）

**作用**：ROS2 节点，100Hz 定时循环，是整个系统的"大脑"。

**代码结构分层**：

```
┌────────────────────────────────────────┐
│  class DeviceNode(Node)                │
│                                        │
│  ├─ __init__()                         │
│  │   ├─ declare_parameter() × 30+      │  ← ROS2参数声明
│  │   ├─ get_parameter() 读取参数       │
│  │   ├─ QoSProfile 配置                │  ← 服务质量策略
│  │   ├─ create_publisher() × 4         │  ← 创建发布者
│  │   ├─ create_subscription() × 2      │  ← 创建订阅者
│  │   ├─ DeviceWrapper.start()          │  ← 启动手柄通信
│  │   └─ create_timer(10ms, callback)   │  ← 100Hz定时器
│  │                                      │
│  ├─ _timer_callback()  ← 100Hz 主循环   │
│  │   ├─ get_state() 读手柄状态          │
│  │   ├─ set_external_force() 写力反馈   │
│  │   ├─ 按钮B：使能/禁用遥操作           │
│  │   ├─ 按钮A：切换夹爪                 │
│  │   ├─ 坐标映射 → 目标位姿             │
│  │   ├─ publish PoseStamped            │
│  │   ├─ publish gripper_command        │
│  │   ├─ publish force_z                │
│  │   └─ publish device_state           │
│  │                                      │
│  ├─ _ee_force_callback()  ← 力反馈接收   │
│  │   ├─ 提取传感器力 + 实际位置          │
│  │   ├─ 低通滤波                       │
│  │   ├─ 阻抗控制计算（弹簧+阻尼）        │
│  │   ├─ 接触检测状态机                  │
│  │   ├─ 触觉脉冲（指数衰减）             │
│  │   ├─ 变刚度（跟踪比）                │
│  │   ├─ 加权混合（传感器力+阻抗力）      │
│  │   ├─ 坐标变换 → 设备坐标系           │
│  │   └─ 存储到 self._external_force_    │
│  │                                      │
│  ├─ _echo_callback()  ← RTT测量         │
│  │   └─ 计算网络往返延迟                │
│  │                                      │
│  └─ _map_device_to_robot_position()    │
│     _map_device_to_robot_orientation() │  ← 坐标系映射
└────────────────────────────────────────┘
```

**坐标映射详解**（最关键的理解点）：

```
手柄坐标系(设备系)                    机器人坐标系(base系)
    Y                                  Y
    ▲                                  ▲
    │                                  │
    └────► X                           └────► X
    
Device X → Robot -Y      (绕 Z 转 90°)
Device Y → Robot X
Device Z → Robot Z       (不变)

数学表达：R_DEV2ROB = [[0, -1, 0],
                     [1,  0, 0],
                     [0,  0, 1]]
```

位置映射公式：
```
delta_device = dev_pos - device_origin          # 手柄相对锚点的位移
scaled = delta_device * position_scale           # 缩放（比如 1mm → 1mm）
delta_robot = R_DEV2ROB @ scaled                # 旋转到机器人坐标系
target_pos = robot_base_pos + delta_robot       # 叠加到机器人当前基准位置
```

**遥操作使能逻辑**（按钮 B）：
- **按住 B**：锚定当前手柄位置为原点，开始控制机器人。之后手柄的相对位移映射为机器人的目标位姿。
- **松开 B**：冻结当前位姿为新的基准点。下次按住 B 时从冻结点继续，**不会跳变**。

**力反馈处理管线**（`_ee_force_callback` 中的完整流程）：

```
从机发来 Wrench 消息
    │
    ├─ force.x/y/z   = 传感器三维力 (N)
    └─ torque.x/y/z  = 实际末端位置 (m)  ← 借用字段传位置
    │
    ▼
① 全零位置过滤（防异常帧）
② 数值微分 → 实际末端速度（低通滤波）
③ 传感器力处理：缩放 → 截断 → 低通滤波
④ 阻抗力计算（仅在遥操作使能时）：
   ├─ 位置误差 = target_pos - actual_pos
   ├─ 径向死区（误差<18mm时忽略）
   ├─ 跟踪比 → 变刚度（K随接触状态变化）
   ├─ 弹簧力 F = K_eff × error
   ├─ 阻尼力 F = C × d_error/dt
   ├─ 接触检测（多条件投票）→ 触觉脉冲
   └─ 截断 + 低通滤波
⑤ 加权混合：
   combined = -0.7×sensor_force + 0.3×impedance_force + pulse
⑥ 安全硬上限（8N，防飞出）
⑦ 坐标变换：robot系 → device系
⑧ 写入 self._external_force_device
    │
    ▼ 在 _timer_callback 中通过 wrapper.set_external_force() 发给手柄
```

---

### 4.4 ROS2 包结构

#### `ros_ws/src/teleop_nodes/setup.py` —— Python 包安装配置

**作用**：告诉 ROS2 怎么安装和运行这个包。

**关键点**：
```python
entry_points={
    'console_scripts': [
        'device_node = teleop_nodes.device_node:main',
        # ros2 run teleop_nodes device_node
        # → 调用 teleop_nodes/device_node.py 里的 main() 函数
    ],
}
```

**data_files**：把 launch/、config/ 等非 Python 文件安装到 `share/` 目录，ROS2 才能找到。

---

#### `ros_ws/src/teleop_nodes/launch/teleop_launch.py` —— 一键启动文件

**作用**：一条命令启动整个节点，不用手动输参数。

```bash
ros2 launch teleop_nodes teleop_launch.py
# 等价于：
ros2 run teleop_nodes device_node --ros-args \
  --params-file /opt/ros/jazzy/share/teleop_nodes/config/device_params.yaml
```

---

#### `ros_ws/src/teleop_msgs/` —— 自定义消息包

**作用**：定义了 `DeviceState.msg`，因为 ROS2 内置消息类型没有同时包含"位置+速度+按钮+姿态+连接状态"的消息。

自定义消息后需要先编译：
```bash
cd ros_ws && colcon build --packages-select teleop_msgs
source install/setup.bash
```

---

## 五、本项目涉及的所有 ROS2 知识点

### 5.1 必会概念（按重要程度排序）

| # | 知识点 | 在本项目中的体现 | 学习优先级 |
|---|--------|---------------|-----------|
| 1 | **Node（节点）** | `class DeviceNode(Node)` | ⭐⭐⭐ |
| 2 | **Topic（话题）** | `/device/pose`, `/end_effector_force` | ⭐⭐⭐ |
| 3 | **Publisher（发布者）** | `self.create_publisher(PoseStamped, '/device/pose', qos)` | ⭐⭐⭐ |
| 4 | **Subscription（订阅者）** | `self.create_subscription(Wrench, '/end_effector_force', callback, qos)` | ⭐⭐⭐ |
| 5 | **Message 类型** | `PoseStamped`, `Wrench`, `Int32`, `Float64`, `DeviceState` | ⭐⭐⭐ |
| 6 | **QoS（服务质量）** | `ReliabilityPolicy.BEST_EFFORT` vs `RELIABLE` | ⭐⭐⭐ |
| 7 | **Parameter（参数）** | `declare_parameter()` + `get_parameter()` | ⭐⭐⭐ |
| 8 | **Timer（定时器）** | `self.create_timer(0.01, self._timer_callback)` → 100Hz | ⭐⭐⭐ |
| 9 | **Launch 文件** | `teleop_launch.py` 一键启动 | ⭐⭐ |
| 10 | **自定义消息** | `teleop_msgs/msg/DeviceState.msg` | ⭐⭐ |
| 11 | **colcon 编译** | `colcon build --packages-select teleop_msgs` | ⭐⭐ |
| 12 | **DDS 跨机通信** | CycloneDDS + `CYCLONEDDS_URI` 配置 | ⭐⭐ |
| 13 | **Executor / spin** | `rclpy.spin(node)` 让节点保持运行 | ⭐⭐ |
| 14 | **ament_index / 包发现** | `get_package_share_directory()` | ⭐ |

### 5.2 每个 ROS2 知识点的代码位置

#### ① Node（节点）
```python
import rclpy
from rclpy.node import Node

class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_node')  # 节点名
```

#### ② Publisher（发布者）
```python
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
self._pub_pose = self.create_publisher(PoseStamped, '/device/pose', qos)

# 发布消息
msg = PoseStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'base'
msg.pose.position.x = 0.3
self._pub_pose.publish(msg)
```

#### ③ Subscription（订阅者）
```python
from geometry_msgs.msg import Wrench

self._sub_ee_force = self.create_subscription(
    Wrench,              # 消息类型
    '/end_effector_force', # 话题名
    self._ee_force_callback,  # 回调函数
    qos_force,           # QoS
)

def _ee_force_callback(self, msg):
    force_x = msg.force.x
    force_y = msg.force.y
    force_z = msg.force.z
```

#### ④ Parameter（参数系统）
```python
# 声明参数（带默认值）
self.declare_parameter('position_scale', 1.0)
self.declare_parameter('robot_home_x', 0.30)

# 读取参数
scale = self.get_parameter('position_scale').value

# 命令行覆盖：
# ros2 run teleop_nodes device_node --ros-args -p position_scale:=1.5
```

#### ⑤ Timer（定时器）
```python
timer_period = 1.0 / 100.0  # 10ms → 100Hz
self._timer = self.create_timer(timer_period, self._timer_callback)

def _timer_callback(self):
    # 每 10ms 执行一次
    pass
```

#### ⑥ QoS（服务质量）
```python
# BEST_EFFORT：尽力而为，不保证送达，延迟低，适合高频实时数据
qos_best_effort = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

# RELIABLE：可靠传输，保证送达，延迟稍高，适合重要指令
qos_reliable = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
```

**BEST_EFFORT vs RELIABLE 的区别**：
- `BEST_EFFORT` = UDP 风格，丢了就丢了，不补发。适合位姿、力反馈这种"最新一帧最重要"的数据。
- `RELIABLE` = TCP 风格，丢了会重传。适合夹爪指令这种"必须送到"的数据。

**坑点**：发布端和订阅端的 QoS 策略要兼容，否则可能收不到消息。

#### ⑦ Message（标准消息类型）

| 消息类型 | 文件 | 字段 | 用途 |
|---------|------|------|------|
| `PoseStamped` | `geometry_msgs` | header + pose.position + pose.orientation | 位姿 |
| `Wrench` | `geometry_msgs` | force(x,y,z) + torque(x,y,z) | 力/力矩 |
| `WrenchStamped` | `geometry_msgs` | header + wrench | 带时间戳的力 |
| `Int32` | `std_msgs` | data | 整数（夹爪指令） |
| `Float64` | `std_msgs` | data | 浮点数（Z轴力） |

#### ⑧ 时间戳与 RTT 测量
```python
# 发布时打时间戳
pose_msg.header.stamp = self.get_clock().now().to_msg()

# 订阅时解析时间戳
original_time = rclpy.time.Time.from_msg(msg.header.stamp)
now = self.get_clock().now()
rtt_ns = (now - original_time).nanoseconds
```

#### ⑨ Launch 文件
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_nodes',
            executable='device_node',
            name='device_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
```

#### ⑩ 自定义消息编译
```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_msgs
source install/setup.bash
```

编译后 Python 才能 `from teleop_msgs.msg import DeviceState as DeviceStateMsg`

#### ⑪ DDS 跨机通信
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/xr/config/cyclone_uri/multi_machine.cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

ROS2 底层用 DDS 实现分布式通信，CycloneDDS 是一种 DDS 实现。XML 配置文件指定了网卡、对端 IP 等。

---

## 六、建议的学习路径

### 阶段 1：ROS2 基础（2-3天）

**目标**：理解 Node、Topic、Publish/Subscribe、Message 的基本概念。

**推荐资源**：
1. [ROS2 官方教程 - Beginner: CLI tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
2. [ROS2 官方教程 - Beginner: Client libraries](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)
3. **必做实验**：
   ```bash
   # 实验1：启动 turtlesim 理解话题
   ros2 run turtlesim turtlesim_node
   ros2 topic list
   ros2 topic echo /turtle1/pose
   ros2 run turtlesim turtle_teleop_key
   
   # 实验2：自己写发布者和订阅者
   # 跟着官方教程写 minimal_publisher.py 和 minimal_subscriber.py
   ```

### 阶段 2：理解本项目架构（1天）

**目标**：不看代码，能画出数据流图，知道每个文件的作用。

**学习方法**：
1. 对照本指南的"系统架构总览"图，在纸上自己画一遍
2. 用 `ros2 topic list` + `ros2 topic hz` + `ros2 topic echo` 观察运行时的话题
3. 修改 `device_node.py` 中的日志，加 print，看执行顺序

### 阶段 3：逐文件精读代码（3-5天）

**建议顺序**：
1. `config_manager.py` → 最简单，理解配置怎么读
2. `inverse_comm.py` → 理解 WebSocket 通信流程（重点看 `main()` 和线程模型）
3. `device_wrapper.py` → 理解数据封装和边沿检测
4. `transformation.py` → 理解四元数连续性
5. `force_controller_1.py` → 理解力计算管线
6. **`device_node.py`** → **最核心最复杂**，分块读：
   - 先读 `__init__()`（参数 + QoS + Publisher/Subscriber 创建）
   - 再读 `_timer_callback()`（100Hz 主循环）
   - 再读 `_ee_force_callback()`（力反馈处理管线）
   - 最后读坐标映射函数

### 阶段 4：动手改代码验证理解（2-3天）

**小实验建议**：
1. 改 `position_scale`，观察机器人运动范围变化
2. 在 `_timer_callback` 里加 `self.get_logger().info()`，观察 100Hz 是否真的在执行
3. 临时注释掉力反馈相关代码，看遥操作是否还能工作（纯位置控制）
4. 用 `ros2 topic pub` 手动发一个 `/end_effector_force` 消息，看主手是否有力反馈

---

## 七、常用调试命令速查

```bash
# ========== 查看话题 ==========
ros2 topic list                          # 列出所有话题
ros2 topic info /device/pose             # 查看话题信息（类型、发布者、订阅者）
ros2 topic echo /device/pose             # 实时打印话题内容
ros2 topic hz /device/pose               # 测量发布频率
ros2 topic delay /device/pose            # 测量发布延迟

# ========== 手动发布测试消息 ==========
ros2 topic pub /end_effector_force geometry_msgs/msg/Wrench \
  "{force: {x: 0.0, y: 0.0, z: 2.0}, torque: {x: 0.3, y: 0.0, z: 0.5}}"

# ========== 查看节点 ==========
ros2 node list
ros2 node info /device_node

# ========== 查看参数 ==========
ros2 param list
ros2 param get /device_node position_scale
ros2 param set /device_node position_scale 1.5

# ========== 编译 ==========
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_msgs
colcon build --packages-select teleop_nodes
source install/setup.bash

# ========== 启动 ==========
ros2 run teleop_nodes device_node
ros2 launch teleop_nodes teleop_launch.py
```

---

## 八、疑难概念速查

### Q1：为什么力话题用的是 `Wrench` 而不是 `WrenchStamped`？

`Wrench` 不含 header（时间戳），`WrenchStamped` 含 header。从端（Franka 工控机）发布的是 `Wrench`，所以订阅端也要匹配。如果类型不匹配，ROS2 会静默收不到消息（不会报错）。

### Q2：`torque` 字段为什么传的是位置？

这是一种**字段借用**（hack）。`Wrench` 有 6 个浮点数字段：force(x,y,z) + torque(x,y,z)。从端把 force 的三个分量用来传力，把 torque 的三个分量用来传实际末端位置。这样只需要一个话题就能同时传力和位置，减少网络开销。

### Q3：为什么 `_ee_force_callback` 和 `_timer_callback` 在不同线程？

ROS2 默认是**单线程 Executor**，所有回调（timer + subscription）都在同一个线程里顺序执行。但如果 subscription 的回调处理太慢（比如力计算管线），就会阻塞 timer 回调，导致位姿发布延迟。

项目注释里提到了改进方向：用 `MultiThreadedExecutor` + `ReentrantCallbackGroup` 让两个回调并行执行。

### Q4：四元数为什么有 xyzw 和 wxyz 两种格式？

| 格式 | 使用方 | 例子 |
|------|--------|------|
| [x, y, z, w] | Haply 设备、scipy、ROS geometry_msgs/Pose | `orientation.x` 先 |
| [w, x, y, z] | MuJoCo、Eigen、部分航天惯例 | `orientation.w` 先 |

本项目里，设备发来的是 xyzw，但 ROS2 的 `PoseStamped.pose.orientation` 也是 xyzw（`orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`），所以转换时要注意。

---

## 九、推荐阅读顺序（文档）

1. **本指南**（你正在看的）—— 先通读一遍，建立全局认知
2. `readme.md` —— 项目的安装、配置、操作说明
3. `CLAUDE.md` —— 项目的架构速览（给 AI 看的，人也看得懂）
4. ROS2 官方教程 —— 边做实验边学
5. 回来看代码 —— 对照本指南逐文件精读

---

> 最后建议：**不要试图一次性看懂所有代码**。先理解架构和数据流，再逐个文件深入。每看完一个文件，回到架构图问自己："这个文件在整个系统里处于哪一环？数据怎么流进流出的？" 这样学习效率最高。
