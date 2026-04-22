```markdown
# Inverse3 遥操作主手端

基于 **Haply Inverse3 + VerseGrip** 触觉主手，通过 ROS 2 DDS 跨机通信，实现对 **Franka FR3** 机械臂的双向力反馈遥操作。

本仓库为 **上位机（主手端）** 代码，负责：
1. 读取 Inverse3 位置/姿态/按钮数据
2. 坐标映射 → 生成机器人目标位姿
3. 接收从机末端力 → 阻抗+脉冲力反馈 → 驱动主手电机

---

## 目录

- [系统架构](#系统架构)
- [项目结构](#项目结构)
- [环境要求](#环境要求)
- [安装](#安装)
- [配置](#配置)
- [启动](#启动)
- [ROS 2 话题](#ros-2-话题)
- [跨机通信配置](#跨机通信配置)
- [关键参数说明](#关键参数说明)
- [操作方式](#操作方式)
- [调试](#调试)

---

## 系统架构

```
┌───────────────────────────────────────────────────────────┐
│                   上位机 (Ubuntu PC)                       │
│                                                           │
│  ┌────────────┐      ┌─────────────────────────────────┐  │
│  │ Inverse3   │◄────►│        device_node.py            │ │
│  │ +VerseGrip │ WS   │  坐标映射 · 力反馈管线 · 夹爪       │ │
│  └────────────┘      └──────┬──────────────────┬────────┘ │
│                        发布 │                  │ 订阅      │
│                             ▼                  ▼          │
│                    /device/pose       /end_effector_force  │
│                    /gripper_command   /pose_echo           │
│                    /device/force_z                         │
└─────────────────────────┬──────────────────┬──────────────┘
                          │  ROS 2 DDS WiFi  │
                          ▼                  ▲
┌─────────────────────────┬──────────────────┬──────────────┐
│                   下位机 (Franka 工控机)                    │
│         pose_relay_node → 控制器 → Franka FR3              │
│         EndEffectorForcePublisher → 力/位置回传             │
└───────────────────────────────────────────────────────────┘
```

**数据流简述：**

| 方向 | 数据 | 话题 | 频率 |
|------|------|------|------|
| 主→从 | 目标位姿 | `/device/pose` | 100 Hz |
| 主→从 | 夹爪指令 | `/gripper_command` | 按需 + 2 Hz 心跳 |
| 主→从 | Z轴期望力 | `/device/force_z` | 20 Hz |
| 从→主 | 末端力+位置 | `/end_effector_force` | ~100 Hz |
| 从→主 | RTT Echo | `/pose_echo` | 100 Hz |

---

## 项目结构

```
.
├── device_node.py                # ROS 2 主节点：坐标映射、力反馈、话题收发
├── Inverse_controller/
│   ├── config/
│   │   ├── leader.toml           # 主手端配置（WebSocket地址、力反馈参数等）
│   │   └── follower.toml         # 从端配置（参考用）
│   ├── config_manager.py         # TOML 配置加载/管理
│   ├── device_wrapper.py         # Inverse3+VerseGrip 高层封装（位置/姿态/按钮）
│   ├── force_controller_1.py     # 力反馈计算器（虚拟墙/滤波/限幅）
│   └── inverse_comm.py           # WebSocket 通信层（与 Inverse3 SDK 交互）
└── requirements.txt              # Python 依赖
```

### 各文件职责

| 文件 | 作用 |
|------|------|
| `device_node.py` | **核心节点**。100Hz 定时器读取主手 → 坐标映射 → 发布位姿；订阅从机力 → 阻抗计算 → 驱动主手力反馈 |
| `inverse_comm.py` | 通过 WebSocket 连接 Inverse3 SDK 服务端，收发设备状态和力指令 |
| `device_wrapper.py` | 封装 `inverse_comm.Device`，提供 `DeviceState` 数据结构（位置/速度/四元数/按钮边沿检测） |
| `force_controller_1.py` | 力反馈基础计算器：虚拟墙、低通滤波、力限幅、死区处理（当前虚拟墙已禁用，仅做外部力透传+滤波） |
| `config_manager.py` | 读取 `config/leader.toml` 或 `follower.toml`，提供 `get_value("network.uri")` 式的层级访问 |

---

## 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Python | 3.12 |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Inverse3 SDK | Haply Desktop API（需后台运行） |

---

## 安装

### 1. 安装docker

```bash
# 卸载旧版本（如有）
sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# 安装前置依赖
sudo apt update && sudo apt install -y ca-certificates curl gnupg

# 添加 Docker 官方 GPG 密钥
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 添加 Docker APT 源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 安装 Docker Engine + Compose 插件
sudo apt update && sudo apt install -y \
  docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# 允许当前用户无 sudo 运行 Docker（需重新登录生效）
sudo usermod -aG docker $USER
newgrp docker

# 验证安装
docker --version
docker compose version
```
后续相关的docker使用参考：https://x2-robot.feishu.cn/wiki/LkLAwTjd9ip6AGkIYxEcnFtoncK
### 2. 工作空间设置
创建文件夹，并拉取haply仓库
```bash
mkdir -p ~/haply_inverse
cd ~/haply_inverse
git clone ssh://git@gitlab.zbl.local:50022/zhaoqiyang/haply_inverse.git
git checkout inverse_docker
```


### 3. 构建镜像并运行

运行creat_image.sh
```bash
./creat_inmage.sh
```

### 4. 启动 Inverse3 SDK

确保 **Haply Desktop** 应用程序在后台运行，它会在本地 `ws://localhost:10001` 开启 WebSocket 服务端。

---

## 配置

### 主手设备配置

编辑 `Inverse_controller/config/leader.toml`：

```toml
[network]
uri = "ws://localhost:10001"   # Inverse3 SDK WebSocket 地址

[device]
force_feedback_enabled = true  # 是否启用力反馈输出到主手
```

### ROS 2 节点参数

`device_node.py` 的所有行为参数都通过 ROS 2 参数系统声明，可以在启动时覆盖：
todo：docker 内的节点是否需要打包？下面内容待修改
```bash
# 示例：修改位置缩放和力反馈增益
ros2 run teleop_nodes device_node --ros-args \
  -p position_scale:=1.5 \
  -p force_scale_x:=0.2 \
  -p force_scale_z:=0.3 \
  -p impedance_k_z:=120.0
```

完整参数列表见 [关键参数说明](#关键参数说明)。

---

## 启动

### 终端 1：启动主手节点
todo: docker 内的节点启动怎么方便，下面内容待修改
```bash
source ~/ros2_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 跨机通信模式（需要先配置好 CycloneDDS，见下文）
export CYCLONEDDS_URI=file:///opt/xr/config/cyclone_uri/multi_machine.cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run teleop_nodes device_node
```

### 验证

```bash
# 另开终端，查看话题列表
ros2 topic list

# 查看位姿发布频率（应约 100Hz）
ros2 topic hz /device/pose

# 查看力反馈数据
ros2 topic echo /end_effector_force
```

---

## ROS 2 话题

### 发布（主手 → 从机）

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/device/pose` | `PoseStamped` | BEST_EFFORT | 机器人目标位姿（base 系），100Hz |
| `/device/state` | `DeviceState` | BEST_EFFORT | 主手原始状态（位置/速度/按钮/姿态） |
| `/gripper_command` | `Int32` | BEST_EFFORT | 夹爪指令：0=打开, 1=闭合 |
| `/device/force_z` | `Float64` | BEST_EFFORT | Z 轴期望力 (N)，20Hz |

### 订阅（从机 → 主手）

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/end_effector_force` | `Wrench` | BEST_EFFORT | 末端力+位置（force=力N, torque=位置m★借用） |
| `/pose_echo` | `PoseStamped` | BEST_EFFORT | RTT 回弹（原始时间戳原样返回） |

---

## 跨机通信配置

### 网络拓扑

| 角色 | 主机名 | IP |
|------|--------|-----|
| 主机（上位机） | `ta-ThinkBook-14-G8-IAL` | `10.150.11.233` |
| 从机（Franka 工控机） | `xr-msi-teleop` | `10.150.11.81` |

### CycloneDDS 配置
todo：docker间通信应该怎么设置比较好
**跨机通信** — `/opt/xr/config/cyclone_uri/multi_machine.cyclonedds.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="0">
        <General>
            <Interfaces>
                <NetworkInterface name="wlp67s0"/>  <!-- 主机无线网卡名 -->
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="10.150.11.81"/>  <!-- 从机 IP -->
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

**本机隔离**（调试用） — `local.cyclonedds.xml`：

```xml
<NetworkInterface name="lo"/>
<AllowMulticast>false</AllowMulticast>
<Peer address="localhost"/>
```

### 切换通信模式

```bash
# 跨机
export CYCLONEDDS_URI=file:///opt/xr/config/cyclone_uri/multi_machine.cyclonedds.xml

# 本机隔离
export CYCLONEDDS_URI=file:///opt/xr/config/cyclone_uri/local.cyclonedds.xml

# 查看当前模式
echo $CYCLONEDDS_URI
```

> ⚠️ `export` 只对当前终端生效。改完配置后必须执行 `ros2 daemon stop`，否则不生效。

### 连通性排查

```bash
# 1. 基础网络测试
ping -c 5 10.150.11.81

# 2. UDP 直连测试（绕过 DDS）
# 从机执行：
nc -u -l -p 9999
# 主机执行：
echo "hello" | nc -u 10.150.11.81 9999

# 3. ROS 2 话题发现
ros2 topic list
ros2 topic echo /test
```

> 如果 UDP 测试不通，说明 WiFi 开启了 AP 隔离，与 ROS 2 配置无关。

---

## 关键参数说明

### 坐标映射

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `position_scale` | 1.0 | 位置缩放系数（主手 1mm → 机器人 1mm×scale） |
| `smoothing_alpha` | 0.15 | 位置平滑滤波系数（越小越平滑，越大响应越快） |
| `robot_home_x/y/z` | 0.30, 0.0, 0.5 | 机器人基准位置 (m) |
| `enable_orientation_tracking` | False | 是否传递姿态增量 |

### 力反馈

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `force_scale_x/y/z` | 0.15, 0.15, 0.25 | 传感器力缩放系数 |
| `max_feedback_force` | 10.0 | 传感器力截断上限 (N) |
| `force_filter_alpha` | 0.5 | 传感器力低通滤波系数 |

### 阻抗控制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `impedance_k_x/y/z` | 50, 50, 100 | 刚度 (N/m) |
| `impedance_c_x/y/z` | 17, 17, 20 | 阻尼 (N·s/m) |
| `sensor_force_weight` | 0.7 | 传感器力混合权重 |
| `impedance_force_weight` | 0.3 | 阻抗力混合权重 |
| `max_impedance_force` | 3.0 | 阻抗力上限 (N) |
| `dead_zone_m` | 0.018 | 径向死区 (m) |

### 接触检测

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `contact_pulse_amplitude` | 1.5 | 触觉脉冲峰值 (N) |
| `contact_pulse_tau` | 0.035 | 脉冲衰减时间常数 (s) |
| `contact_cooldown_s` | 0.5 | 脉冲最小间隔 (s) |
| `combined_force_hard_limit` | 8.0 | 合力安全硬上限 (N) |

### 变刚度

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `k_min_scale` | 1.0 | 自由空间刚度倍率 |
| `k_max_scale` | 2.0 | 硬接触刚度倍率 |
| `tracking_window_size` | 20 | 跟踪比滑动窗口 (帧) |

---

## 操作方式

| 按钮 | 功能 |
|------|------|
| **B 键（拇指侧）** | 按住 = 遥操作使能，松开 = 冻结当前位姿 |
| **A 键（食指侧）** | 按一下 = 切换夹爪开/合 |

**操作流程：**

1. 启动节点后，机器人处于 Home 位置，主手不控制机器人
2. **按住 B 键** → 锚定当前主手位置为原点，开始遥操作
3. 移动主手 → 机器人跟随运动，手指感受到力反馈
4. **松开 B 键** → 机器人停在当前位置
5. 再次按住 B 键 → 从当前冻结位置继续，不会跳变
6. 按 A 键 → 夹爪开/合切换

---

## 调试

### 日志文件

节点运行时自动在 `./force_debug/` 目录生成：
| 文件 | 内容 |
|------|------|
| `force_debug_*.txt` | 传感器原始力、设备输出力、回调计数（每 50 帧记一条） |
| `rtt_log_*.csv` | 每条 RTT 测量值 (ms)，格式：`seq,rtt_ms` |

### RTT 统计

节点每 500 条自动打印 RTT 统计：

```
📡 网络RTT统计 (最近500条): 均值=13.12ms, 中位=6.89ms, 最大=78.59ms
```

### 常用调试命令

```bash
# 查看位姿发布频率
ros2 topic hz /device/pose

# 查看力数据
ros2 topic echo /end_effector_force

# 查看发布延迟
ros2 topic delay /device/pose

# 网络延迟测试
ping -i 0.005 -c 1000 10.150.11.81
```

---

## 性能指标（实测参考）

| 指标 | 数值 |
|------|------|
| 位姿发布频率 | ~100 Hz |
| 力反馈接收频率 | ~100 Hz |
| 网络 RTT（WiFi） | 中位 6.9ms，最大 78.6ms |
| Ping 延迟 | 5–16ms |
| 总链路延迟（含 Franka 处理） | ~25ms |
```