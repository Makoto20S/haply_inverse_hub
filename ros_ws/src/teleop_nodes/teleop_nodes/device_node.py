# 文件：~/ros2_ws/src/teleop_nodes/teleop_nodes/device_node.py
"""
device_node.py  —— 跨机遥操作版本 v2.1

v2.1 改动：
  - 修复调试文件路径：改用 ~/force_debug/ 目录（保证可写）
  - 修复力反馈接收：QoS 改为 10/RELIABLE（兼容多数发布端）
  - 新增回调计数器：确认力话题是否真的收到了数据
  - 新增 QoS 自动探测日志
"""

import sys
import os
import gc



import numpy as np
import datetime
import rclpy
import time as _time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from teleop_msgs.msg import DeviceState as DeviceStateMsg
from geometry_msgs.msg import PoseStamped, WrenchStamped, Wrench
from std_msgs.msg import Int32, Float64  # ★ 新增：用于夹爪命令
from collections import deque

sys.path.insert(0, '/HAPLY_INVERSE/Inverse_controller')
from device_wrapper import DeviceWrapper, ButtonID
from config_manager import ConfigMode


# ============================================================
# 四元数工具函数
# ============================================================

def quat_multiply(q1, q2):
    """四元数乘法，格式 [w, x, y, z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conjugate(q):
    """四元数共轭，格式 [w, x, y, z]"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """四元数归一化"""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


class DeviceNode(Node):

    # ============================================================
    # 坐标系映射关系
    # ============================================================

    R_DEV2ROB = np.array([
        [0, -1, 0],
        [1,  0, 0],
        [0,  0, 1],
    ], dtype=np.float64)

    R_ROB2DEV = R_DEV2ROB.T

    Q_DEV2ROB_WXYZ = np.array([0.70710678, 0.0, 0.0, 0.70710678])

    def __init__(self):
        super().__init__('device_node')

        # ============================================================
        # ROS2 参数
        # ============================================================
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('startup_wait', 2.0)
        self.declare_parameter('position_scale', 1.0)

        self.declare_parameter('robot_home_x', 0.30)
        self.declare_parameter('robot_home_y', 0.0)
        self.declare_parameter('robot_home_z', 0.5)
        self.declare_parameter('robot_home_qx', 0.999)
        self.declare_parameter('robot_home_qy', 0.0)
        self.declare_parameter('robot_home_qz', 0.0)
        self.declare_parameter('robot_home_qw', 0.0)

        self.declare_parameter('force_scale_x', 0.15)
        self.declare_parameter('force_scale_y', 0.15)
        self.declare_parameter('force_scale_z', 0.25)
        self.declare_parameter('max_feedback_force', 10.0)
        self.declare_parameter('force_filter_alpha', 0.5)

        self.declare_parameter('smoothing_alpha', 0.15)

        self.declare_parameter('enable_orientation_tracking', False)
        self.declare_parameter('orientation_scale', 1.0)

        # ★★★ 力话题名参数（方便调试时切换）
        self.declare_parameter('force_topic', '/end_effector_force')
        # ================================================================
        # ★ NEW: 阻抗控制参数，declare_parameter让节点可以从外部接收配置值，ros2 run teleop_nodes device_node --ros-args -p mpedance_k_x:=300
        # ================================================================
        self.declare_parameter('impedance_k_x', 50.0)   # 刚度 N/m
        self.declare_parameter('impedance_k_y', 50.0)
        self.declare_parameter('impedance_k_z', 100.0)
        self.declare_parameter('impedance_c_x', 17.0)    # 阻尼 N·s/m
        self.declare_parameter('impedance_c_y', 17.0)
        self.declare_parameter('impedance_c_z', 20.0)
        self.declare_parameter('sensor_force_weight', 0.7)     # 传感器力权重
        self.declare_parameter('impedance_force_weight', 0.3)  # 阻抗力权重
        self.declare_parameter('max_impedance_force', 3.0)     # 阻抗力上限 N
        self.declare_parameter('enable_impedance_feedback', True)
        self.declare_parameter('velocity_filter_alpha', 0.2)  # 速度滤波Alpha
        self.declare_parameter('impedance_filter_alpha', 0.3) # 阻抗力滤波Alpha

        # ================================================================
        # ★ NEW: 死区 + 接触检测 + 变刚度参数
        # ================================================================
        self.declare_parameter('dead_zone_m', 0.018)              # 死区 18mm
        self.declare_parameter('contact_pulse_amplitude', 1.5)    # 脉冲峰值 N（不经过权重缩放，直接作用）
        self.declare_parameter('contact_pulse_tau', 0.035)        # 脉冲衰减常数 35ms
        self.declare_parameter('contact_error_rate_threshold', 0.04)  # 误差增长速率阈值 m/s
        self.declare_parameter('contact_force_step_threshold', 2.5)   # 传感器脉冲力阶跃阈值 N
        self.declare_parameter('contact_cooldown_s', 0.5)         # 脉冲最小间隔 s
        self.declare_parameter('k_min_scale', 1.0)                # 变刚度：自由空间 K 倍率
        self.declare_parameter('k_max_scale', 2.0)                # 变刚度：硬接触 K 倍率
        self.declare_parameter('tracking_window_size', 20)        # 跟踪比计算窗口（帧数）
        self.declare_parameter('combined_force_hard_limit', 8.0)  # 合力安全硬上限 N

        # ★ NEW: Z 轴期望力发送参数 ========================================
        self.declare_parameter('force_z_desired', -3.0)        # 期望力(N)，负值=向下推
        self.declare_parameter('force_z_topic', '/device/force_z')  # 话题名
        self.declare_parameter('force_z_publish_divider', 5)   # 每N个定时器周期发一次(100Hz/5=20Hz)
        self.declare_parameter('enable_force_z_publish', True) # 是否启用
 

        # ---- 读取参数 ----
        publish_rate = self.get_parameter('publish_rate_hz').value
        startup_wait = self.get_parameter('startup_wait').value
        self._position_scale = self.get_parameter('position_scale').value
        self._smoothing_alpha = self.get_parameter('smoothing_alpha').value

        self._force_scale_xyz = np.array([
            self.get_parameter('force_scale_x').value,
            self.get_parameter('force_scale_y').value,
            self.get_parameter('force_scale_z').value,
        ])
        self._max_feedback_force = self.get_parameter('max_feedback_force').value
        self._force_filter_alpha = self.get_parameter('force_filter_alpha').value

        self._enable_ori_tracking = self.get_parameter('enable_orientation_tracking').value
        self._orientation_scale = self.get_parameter('orientation_scale').value

        force_topic = self.get_parameter('force_topic').value
        # ================================================================
        # ★ NEW: 读取阻抗参数
        # ================================================================
        self._impedance_K = np.array([
            self.get_parameter('impedance_k_x').value,
            self.get_parameter('impedance_k_y').value,
            self.get_parameter('impedance_k_z').value,
        ])
        self._impedance_C = np.array([
            self.get_parameter('impedance_c_x').value,
            self.get_parameter('impedance_c_y').value,
            self.get_parameter('impedance_c_z').value,
        ])
        self._sensor_weight = self.get_parameter('sensor_force_weight').value
        self._impedance_weight = self.get_parameter('impedance_force_weight').value
        self._max_impedance_force = self.get_parameter('max_impedance_force').value
        self._enable_impedance = self.get_parameter('enable_impedance_feedback').value
        self._velocity_filter_alpha = self.get_parameter('velocity_filter_alpha').value
        self._impedance_filter_alpha = self.get_parameter('impedance_filter_alpha').value

        # ★ NEW: 读取死区 + 接触检测 + 变刚度参数
        self._dead_zone = self.get_parameter('dead_zone_m').value
        self._pulse_amplitude = self.get_parameter('contact_pulse_amplitude').value
        self._pulse_tau = self.get_parameter('contact_pulse_tau').value
        self._error_rate_threshold = self.get_parameter('contact_error_rate_threshold').value
        self._force_step_threshold = self.get_parameter('contact_force_step_threshold').value
        self._contact_cooldown = self.get_parameter('contact_cooldown_s').value
        self._k_min_scale = self.get_parameter('k_min_scale').value
        self._k_max_scale = self.get_parameter('k_max_scale').value
        self._tracking_window = self.get_parameter('tracking_window_size').value
        self._combined_hard_limit = self.get_parameter('combined_force_hard_limit').value
        # ★ NEW: 读取 Z 轴力参数
        self._force_z_desired = self.get_parameter('force_z_desired').value
        self._force_z_topic = self.get_parameter('force_z_topic').value
        self._force_z_divider = self.get_parameter('force_z_publish_divider').value
        self._enable_force_z = self.get_parameter('enable_force_z_publish').value


        self._robot_home_pos = np.array([
            self.get_parameter('robot_home_x').value,
            self.get_parameter('robot_home_y').value,
            self.get_parameter('robot_home_z').value,
        ])
        self._robot_home_quat_xyzw = np.array([
            self.get_parameter('robot_home_qx').value,
            self.get_parameter('robot_home_qy').value,
            self.get_parameter('robot_home_qz').value,
            self.get_parameter('robot_home_qw').value,
        ])
        self._robot_home_quat_wxyz = np.array([
            self._robot_home_quat_xyzw[3],
            self._robot_home_quat_xyzw[0],
            self._robot_home_quat_xyzw[1],
            self._robot_home_quat_xyzw[2],
        ])

        self.get_logger().info(
            f'参数: rate={publish_rate}Hz, pos_scale={self._position_scale}, '
            f'smooth_alpha={self._smoothing_alpha}'
        )
        self.get_logger().info(
            f'力反馈: scale_xyz={self._force_scale_xyz}, '
            f'max={self._max_feedback_force}N, filter_alpha={self._force_filter_alpha}'
        )
        self.get_logger().info(
            f'力话题: {force_topic}'
        )
        self.get_logger().info(
            f'姿态追踪: {"开启" if self._enable_ori_tracking else "关闭"}'
        )
        self.get_logger().info(
            f'Franka Home: pos={self._robot_home_pos}, '
            f'quat_xyzw={self._robot_home_quat_xyzw}'
        )

        # ============================================================
        # QoS
        # ============================================================
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ★★★ 力反馈专用 QoS ★★★
        # 用 depth=10 + RELIABLE + VOLATILE
        # 这是最通用的组合，兼容绝大多数发布端
        # 如果还不行，改成 BEST_EFFORT 再试
        qos_force = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ★ 匹配从机发布端
            history=HistoryPolicy.KEEP_LAST,
        )

        # ============================================================
        # 发布者
        # ============================================================
        self._pub_state = self.create_publisher(
            DeviceStateMsg, '/device/state', qos_best_effort
        )
        # self._pub_pose = self.create_publisher(
        #     PoseStamped, '/device/pose', qos_reliable
        # )
        self._pub_pose = self.create_publisher(
            PoseStamped, '/device/pose', qos_best_effort
        )
        # ★★★ 新增：夹爪命令发布者 ★★★
        self._pub_gripper = self.create_publisher(
            Int32, '/gripper_command', qos_best_effort
        )
        self.get_logger().info('夹爪话题: /gripper_command (QoS: BEST_EFFORT)')
        # ★ NEW: Z 轴期望力发布者
        self._pub_force_z = self.create_publisher(
            Float64, self._force_z_topic, qos_best_effort
        )
        self._force_z_pub_count = 0  # 发布计数（用于日志）
        self.get_logger().info(
            f'Z轴力话题: {self._force_z_topic} '
            f'(期望力={self._force_z_desired}N, '
            f'频率={100.0/self._force_z_divider:.0f}Hz, '
            f'启用={self._enable_force_z})'
        )

        # ============================================================
        # 设备初始化
        # ============================================================
        self.get_logger().info('正在初始化 DeviceWrapper ...')
        self._wrapper = DeviceWrapper(
            config_mode=ConfigMode.LEADER,
            startup_wait=startup_wait,
        )
        connected = self._wrapper.start()
        if connected:
            self.get_logger().info('设备连接成功')
        else:
            self.get_logger().warn('设备未连接，但节点仍将运行')

        # ============================================================
        # ★★★ 订阅从机末端力反馈 ★★★
        # ============================================================
        self.get_logger().info(
            f'正在订阅力话题: {force_topic} (QoS: BSET_EFFORT, depth=10)'
        )
        self._sub_ee_force = self.create_subscription(
            Wrench,          # ★★★ 改为 Wrench，不是 WrenchStamped ★★★
            force_topic,
            self._ee_force_callback,
            qos_force,
        )

        # ★★★ 力反馈状态 + 诊断计数器 ★★★
        self._external_force_device = [0.0, 0.0, 0.0]
        self._filtered_force_robot = np.zeros(3)
        self._raw_force_robot = np.zeros(3)     # 保存原始力（用于显示）
        self._force_callback_count = 0          # 回调被调用的总次数
        self._force_callback_first = True       # 是否是第一次收到

        # ============================================================
        # ★ NEW: 阻抗计算所需的状态变量
        # ============================================================
        # 从机实际末端位置（robot base 系），初始化为 home
        self._actual_robot_pos = self._robot_home_pos.copy()
        self._actual_pos_received = False          # 是否收到过实际位置
        self._actual_pos_timestamp = 0.0
        # ★ NEW: 零值过滤——记录上一帧有效位置，用于替换非法的全零帧
        self._last_valid_pos = self._robot_home_pos.copy()
        self._zero_pos_count = 0                   # 统计被过滤掉的全零帧次数
        # 数值微分 → 估算实际末端速度
        self._prev_actual_pos = self._robot_home_pos.copy()
        self._prev_actual_time = 0.0
        self._actual_robot_vel = np.zeros(3)
        # 阻抗力滤波（独立于传感器力滤波）
        self._filtered_impedance_force = np.zeros(3)
        # self._impedance_filter_alpha = 0.3  # 滤波参数，越小越平滑，越大响应越快
        # 速度滤波参数，越小越平滑，越大响应越快
        # self._velocity_filter_alpha = 0.2  

        # ============================================================
        # ★ NEW: 死区/接触检测/变刚度 状态变量
        # ============================================================
        # 误差率阻尼用
        self._prev_pos_error = np.zeros(3)
        self._filtered_d_error = np.zeros(3)    # ★ NEW
        # 接触状态机
        self._contact_state = 'free'        # 'free' 或 'contact'
        self._pulse_start_time = 0.0        # 脉冲触发时刻
        self._pulse_direction = np.zeros(3) # 脉冲方向（归一化）
        self._last_contact_time = 0.0       # 上次触发时刻（冷却用）
        self._prev_sensor_force_mag = 0.0   # 前一帧传感器力幅度
        self._impedance_frame_count = 0     # 帧计数，跳过前几帧
        # 变刚度：跟踪比
        # self._tracking_delta_actual = []    # 窗口内 |Δactual|
        # self._tracking_delta_target = []    # 窗口内 |Δtarget|
        self._tracking_delta_actual = deque(maxlen=self._tracking_window)
        self._tracking_delta_target = deque(maxlen=self._tracking_window)
        self._prev_actual_for_tracking = self._robot_home_pos.copy()
        self._prev_target_for_tracking = self._robot_home_pos.copy()
        self._current_tracking_ratio = 1.0  # 1.0=完美跟随（自由空间）


        # ============================================================
        # 遥操作状态
        # ============================================================
        self._enabled = False
        self._device_origin_pos = np.zeros(3)
        self._device_origin_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        self._robot_base_pos = self._robot_home_pos.copy()
        self._robot_base_quat_wxyz = self._robot_home_quat_wxyz.copy()
        self._current_target_pos = self._robot_home_pos.copy()
        self._current_target_quat_xyzw = self._robot_home_quat_xyzw.copy()
        self._smoothed_pos = self._robot_home_pos.copy()
        self._prev_buttons = [False, False, False]
        self._loop_count = 0
        # ★★★ 新增：夹爪状态变量 ★★★
        self._gripper_closed = False  # 当前夹爪状态：False=打开(0), True=闭合(1)

        # ============================================================
        # ★★★ 力反馈调试文件 ★★★
        # ============================================================
        self._force_debug_file = None
        self._force_debug_enabled = True
        self._force_debug_counter = 0
        self._force_debug_interval = 50
        if self._force_debug_enabled:
            self._init_force_debug_file()

        # ============================================================
        # 定时器
        # ============================================================
        timer_period = 1.0 / publish_rate
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('device_node v2.1 启动完成')
        self.get_logger().info(
            '=' * 50 + '\n'
            '  ⏳ 等待力话题数据...\n'
            f'  话题: {force_topic}\n'
            '  如果10秒后仍显示"尚未收到"，请检查：\n'
            '  1. 从机节点是否在运行\n'
            '  2. ros2 topic list 中是否能看到该话题\n'
            '  3. ros2 topic info <话题名> 查看 QoS\n'
            + '=' * 50
        )
        # ★★★ 新增：夹爪初始状态提示 ★★★
        self.get_logger().info(
            f'🤖 夹爪初始状态: {"🔴 闭合" if self._gripper_closed else "🟢 打开"} '
            f'(按钮A切换)'
        )

        # ============================================================
        # ★★★ 新增：RTT 测量（Echo 回环）★★★
        # ============================================================
        self._rtt_measurements = []         # 存所有 RTT 样本（ms）
        self._rtt_log_interval = 500        # 每 500 个样本打印一次统计
        self._rtt_debug_file = None
        # 初始化 RTT 调试文件
        try:
            debug_dir = '/HAPLY_INVERSE/force_debug'#从虚拟环境迁移至docker，此处改为绝对路径，确保有写权限
            os.makedirs(debug_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            rtt_filepath = os.path.join(debug_dir, f'rtt_log_{timestamp}.csv')
            self._rtt_debug_file = open(rtt_filepath, 'w', buffering=1)
            self._rtt_debug_file.write("seq,rtt_ms\n")
            self.get_logger().info(f'✅ RTT 日志文件: {rtt_filepath}')
        except Exception as e:
            self.get_logger().error(f'❌ 无法创建RTT日志: {e}')
        # 订阅 Franka 回弹的 echo 话题
        self._sub_echo = self.create_subscription(
            PoseStamped,
            '/pose_echo',
            self._echo_callback,
            qos_best_effort,   # 和发布端一致
        )
        self.get_logger().info('已订阅 /pose_echo（RTT 测量）')

        gc.disable()

    # ============================================================
    # ★★★ 力反馈调试文件初始化（修复路径）★★★
    # ============================================================

    def _init_force_debug_file(self):
        """
        初始化力反馈调试文件

        ★ 关键修复：不用 __file__ 目录（install目录无写权限）
          改用 ~/force_debug/ 目录，保证可写
        """
        try:
            # ★★★ 修复：使用固定路径 ★★★
            debug_dir = '/HAPLY_INVERSE/force_debug'
            os.makedirs(debug_dir, exist_ok=True)  # 自动创建目录

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'force_debug_{timestamp}.txt'
            filepath = os.path.join(debug_dir, filename)

            self._force_debug_file = open(filepath, 'w', buffering=1)
            self._force_debug_file.write(
                f"# 力反馈调试数据 - {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                "# 列: 时间戳, raw_x(N), raw_y(N), raw_z(N), "
                "dev_x(N), dev_y(N), dev_z(N), callback_count\n"
                "#\n"
                "# raw = 从机robot base系原始力\n"
                "# dev = 经过缩放+滤波+坐标变换后喂给设备的力\n"
                "#\n"
            )
            self._force_debug_file.flush()

            self.get_logger().info(f'✅ 调试文件已创建: {filepath}')

            # 写入一条测试行，确认文件确实可写
            self._force_debug_file.write(
                f"# TEST: 文件写入正常 at {datetime.datetime.now()}\n"
            )
            self._force_debug_file.flush()

        except Exception as e:
            self.get_logger().error(f'❌ 无法创建调试文件: {e}')
            self._force_debug_enabled = False

    # ============================================================
    # ★★★ 力反馈记录 ★★★
    # ============================================================

    def _log_force_data(self, raw_force, processed_force):
        """记录力反馈数据到文件"""
        if not self._force_debug_enabled or self._force_debug_file is None:
            return

        self._force_debug_counter += 1

        if self._force_debug_counter % self._force_debug_interval == 0:
            try:
                ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                raw = raw_force
                dev = processed_force
                line = (
                    f"{ts}, "
                    f"{raw[0]:+8.3f}, {raw[1]:+8.3f}, {raw[2]:+8.3f}, "
                    f"{dev[0]:+8.4f}, {dev[1]:+8.4f}, {dev[2]:+8.4f}, "
                    f"{self._force_callback_count}\n"
                )
                self._force_debug_file.write(line)
                self._force_debug_file.flush()
            except Exception as e:
                self.get_logger().warn(f'写入调试数据失败: {e}')

    # ============================================================
    # ★★★ 力反馈回调（完整处理管线 + 诊断）★★★
    # ============================================================

    def _ee_force_callback(self, msg):
            """
            从机末端力+位置 → 阻抗力计算 → 加权混合 → 喂给设备
            从机 Wrench 消息格式约定：
            force.x/y/z  = 末端传感器三维力 (N)
            torque.x/y/z = 末端实际位置 x/y/z (m)  ★ 借用 torque 字段传位置
            """
            # ---- 诊断计数器 ----
            self._force_callback_count += 1
            if self._force_callback_first:
                self._force_callback_first = False
                self.get_logger().info(
                    f'✅ 首次收到力+位置数据！'
                    f'force=[{msg.force.x:.2f}, {msg.force.y:.2f}, {msg.force.z:.2f}] N  '
                    f'actual_pos=[{msg.torque.x:.4f}, {msg.torque.y:.4f}, {msg.torque.z:.4f}] m'
                )
            if self._force_callback_count % 1000 == 0:
                self.get_logger().info(
                    f'力回调统计: 已收到 {self._force_callback_count} 次'
                )
            # ① 提取原始传感器力 (force 字段)
            raw_force = np.array([msg.force.x, msg.force.y, msg.force.z])
            self._raw_force_robot = raw_force.copy()
            # ② 提取实际末端位置 (torque 字段，借用传位置)
            now = _time.time()
            new_pos_raw = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
            # ★ NEW: 全零位置过滤
            # 机械臂正常工作时坐标不可能同时全为零（base 系原点在机械臂底座内部）
            # 三轴同时为零判定为无效帧，用上一帧有效值代替，避免引入虚假的位置跳变
            if np.allclose(new_pos_raw, 0.0, atol=1e-9):
                self._zero_pos_count += 1
                # # 每过滤 50 次打印一次警告，避免刷屏
                # if self._zero_pos_count % 50 == 1:
                #     self.get_logger().warn(
                #         f'⚠️ 收到全零位置数据，已过滤（累计 {self._zero_pos_count} 次）'
                #         f'，使用上一帧有效值 {self._last_valid_pos}'
                #     )
                new_pos = self._last_valid_pos.copy()
            else:
                new_pos = new_pos_raw
                self._last_valid_pos = new_pos.copy()   # 记录本帧作为下次备用
            # 数值微分估算实际末端速度（用于阻尼项）
            dt = now - self._prev_actual_time if self._prev_actual_time > 0 else 0.01
            if dt > 0.001:  # 防止除零
                raw_vel = (new_pos - self._prev_actual_pos) / dt
                # 一阶低通滤波平滑速度（数值微分噪声大）
                alpha_vel = self._velocity_filter_alpha
                self._actual_robot_vel = (
                    alpha_vel * raw_vel + (1.0 - alpha_vel) * self._actual_robot_vel
                )
            self._prev_actual_pos = self._actual_robot_pos.copy()
            self._prev_actual_time = now
            self._actual_robot_pos = new_pos
            self._actual_pos_received = True
            self._actual_pos_timestamp = now
            # ③ 传感器力处理管线（缩放 → 截断 → 滤波）—— 和之前一样
            scaled_force = raw_force * self._force_scale_xyz
            force_mag = np.linalg.norm(scaled_force)
            if force_mag > self._max_feedback_force:
                scaled_force = scaled_force * (self._max_feedback_force / force_mag)
            alpha = self._force_filter_alpha
            self._filtered_force_robot = (
                alpha * scaled_force + (1.0 - alpha) * self._filtered_force_robot
            )

            # ④ 计算阻抗力（robot base 系）—— 死区 + 误差率阻尼 + 接触脉冲 + 变刚度
            impedance_force_robot = np.zeros(3)
            contact_pulse_robot = np.zeros(3)   # ★ 脉冲单独计算，不经过权重
            if self._enable_impedance and self._actual_pos_received and self._enabled:
                self._impedance_frame_count += 1
                # ---- 位置误差 ----
                pos_error = self._current_target_pos - self._actual_robot_pos
                pos_error_mag = np.linalg.norm(pos_error)
                # ---- 误差变化率（滤波 + 钳位）----
                if dt > 0.001:
                    raw_d_error = (pos_error - self._prev_pos_error) / dt
                else:
                    raw_d_error = np.zeros(3)
                # ★ FIX-1a: 幅度钳位——防止 dt 抖动导致的尖峰
                #   d_error 物理含义是"误差增长速度(m/s)"
                #   正常操作不可能超过 0.5 m/s，超过的都是噪声
                DE_CLAMP = 0.5  # m/s
                raw_d_error = np.clip(raw_d_error, -DE_CLAMP, DE_CLAMP)
                # ★ FIX-1b: 独立低通滤波——alpha 越小越平滑
                #   用 0.08（比 impedance_filter_alpha=0.3 低很多）
                #   意味着只有约 8% 的新值进入，大幅消除毛刺
                de_alpha = 0.08
                self._filtered_d_error = (
                    de_alpha * raw_d_error
                    + (1.0 - de_alpha) * self._filtered_d_error
                )
                d_error = self._filtered_d_error
                d_error_mag = np.linalg.norm(d_error)
                # ---- 死区处理（改用径向死区，消除轴独立开关问题）----
                # ★ FIX-3: 径向死区——按误差矢量的模做门槛，不按轴分开判断
                #   这样斜着移动时不会有某个轴反复开关的问题
                dz = self._dead_zone
                if pos_error_mag > dz:
                    # 方向不变，模减去死区
                    effective_error = pos_error * ((pos_error_mag - dz) / pos_error_mag)
                else:
                    effective_error = np.zeros(3)
                # ---- 跟踪比 → 变刚度 ----
                delta_actual_s = np.linalg.norm(
                    new_pos - self._prev_actual_for_tracking
                )
                delta_target_s = np.linalg.norm(
                    self._current_target_pos - self._prev_target_for_tracking
                )
                # self._tracking_delta_actual.append(delta_actual_s)
                # self._tracking_delta_target.append(delta_target_s)
                # 添加元素：自动保持窗口大小
                self._tracking_delta_actual.append(delta_actual_s)
                self._tracking_delta_target.append(delta_target_s)
                # 保持窗口大小
                # while len(self._tracking_delta_actual) > self._tracking_window:
                #     self._tracking_delta_actual.pop(0)
                #     self._tracking_delta_target.pop(0)
                
                self._prev_actual_for_tracking = new_pos.copy()
                self._prev_target_for_tracking = self._current_target_pos.copy()
                # 无需手动删除最老元素，deque 自动处理
                # sum() 仍然可用
                sum_da = sum(self._tracking_delta_actual)
                sum_dt_trk = sum(self._tracking_delta_target)
                if sum_dt_trk > 0.0005:
                    raw_tr = min(1.0, sum_da / sum_dt_trk)
                else:
                    raw_tr = 1.0
                # ★ FIX-2: 跟踪比低通滤波——防止 k_scale 帧间振荡
                tr_alpha = 0.05  # 非常平滑，约需 20 帧（200ms）才追上真值
                self._current_tracking_ratio = (
                    tr_alpha * raw_tr
                    + (1.0 - tr_alpha) * self._current_tracking_ratio
                )
                k_scale = (
                    self._k_min_scale
                    + (self._k_max_scale - self._k_min_scale)
                    * (1.0 - self._current_tracking_ratio)
                )
                K_eff = self._impedance_K * k_scale
                # ---- 弹簧力 ----
                F_spring = K_eff * effective_error
                # ---- 阻尼力（使用滤波后的 d_error）----
                F_damping = self._impedance_C * d_error
                # ============================================================
                # 接触检测状态机（多条件投票 + 触觉脉冲）
                # ============================================================
                F_pulse = np.zeros(3)
                # 跳过前 10 帧（初始误差率不可靠）
                if self._impedance_frame_count > 10:
                    sensor_force_mag = np.linalg.norm(raw_force)
                    force_step = sensor_force_mag - self._prev_sensor_force_mag
                    # 三个检测条件
                    cond_A = pos_error_mag > dz                       # 误差超出死区
                    cond_B = d_error_mag > self._error_rate_threshold # 误差急剧增大
                    cond_C = force_step > self._force_step_threshold  # 传感器力阶跃，大权重
                    votes = int(cond_A) + int(cond_B) + 2*int(cond_C)
                    cooldown_ok = (now - self._last_contact_time) > self._contact_cooldown
                    # 状态转移
                    if self._contact_state == 'free':
                        if votes >= 3 and cooldown_ok:
                            # ★ 触发接触！
                            self._contact_state = 'contact'
                            self._pulse_start_time = now
                            self._last_contact_time = now
                            if pos_error_mag > 1e-6:
                                self._pulse_direction = pos_error / pos_error_mag
                            else:
                                self._pulse_direction = np.array([0.0, 0.0, 1.0])
                            self.get_logger().info(
                                f'🔴 接触！votes={votes} '
                                f'[A={cond_A},B={cond_B},C={cond_C}] '
                                f'err={pos_error_mag*1000:.1f}mm '
                                f'tr={self._current_tracking_ratio:.2f}'
                            )
                    elif self._contact_state == 'contact':
                        # 脱离判定：误差回到死区一半以内
                        if pos_error_mag < dz * 0.5:
                            self._contact_state = 'free'
                            self._pulse_direction = np.zeros(3)
                    self._prev_sensor_force_mag = sensor_force_mag
                # 计算脉冲力（指数衰减，不滤波不加权）
                if self._pulse_start_time > 0:
                    elapsed = now - self._pulse_start_time
                    # 5 倍 tau 后脉冲衰减到 <1%，自动停止
                    if elapsed < self._pulse_tau * 5.0:
                        pulse_val = self._pulse_amplitude * np.exp(
                            -elapsed / self._pulse_tau
                        )
                        F_pulse = pulse_val * self._pulse_direction
                    else:
                        self._pulse_start_time = 0.0  # 脉冲结束
                contact_pulse_robot = F_pulse
                # ---- 更新前一帧误差 ----
                self._prev_pos_error = pos_error.copy()
                # ---- 弹簧+阻尼 → 截断 → 低通滤波 ----
                raw_sd = F_spring + F_damping
                sd_mag = np.linalg.norm(raw_sd)
                if sd_mag > self._max_impedance_force:
                    raw_sd = raw_sd * (self._max_impedance_force / sd_mag)
                a_imp = self._impedance_filter_alpha
                self._filtered_impedance_force = (
                    a_imp * raw_sd
                    + (1.0 - a_imp) * self._filtered_impedance_force
                )
                impedance_force_robot = self._filtered_impedance_force
            # ⑤ 加权混合 + 脉冲（脉冲不经过权重，直接叠加）
            combined_force_robot = (
                -self._sensor_weight * self._filtered_force_robot
                + self._impedance_weight * impedance_force_robot
                + contact_pulse_robot
            )
            # ★ 安全硬上限（防止合力过大导致主手飞出）
            combined_mag = np.linalg.norm(combined_force_robot)
            if combined_mag > self._combined_hard_limit:
                combined_force_robot = combined_force_robot * (
                    self._combined_hard_limit / combined_mag
                )
            # ⑥ 坐标变换：robot base 系 → device 系
            f_device = -self.R_ROB2DEV @ combined_force_robot
            # ⑦ 喂给设备
            self._external_force_device = [
                float(f_device[0]),
                float(f_device[1]),
                float(f_device[2]),
            ]
            # ⑧ 记录到调试文件
            self._log_force_data(raw_force, self._external_force_device)

    # ============================================================
    # ★★★ Echo 回调：测量网络 RTT ★★★
    # ============================================================
    def _echo_callback(self, msg: PoseStamped):
        """
        收到 Franka 回弹的 echo 消息，计算 RTT
        
        原理：
          msg.header.stamp = 当初 master 发布 pose 时写入的时间戳（master 的时钟）
          self.get_clock().now() = 现在 master 的时间（同一个时钟）
          差值 = 消息在网络上跑了一个来回的时间
        """
        # 解析原始发送时间（这是 master 自己的时钟打的）
        original_time = rclpy.time.Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        # 计算 RTT（纳秒转毫秒）
        rtt_ns = (now - original_time).nanoseconds
        rtt_ms = rtt_ns / 1e6
        # 过滤异常值（负值或超过 1 秒的肯定有问题）
        if rtt_ms < 0 or rtt_ms > 1000:
            return
        self._rtt_measurements.append(rtt_ms)
        seq = len(self._rtt_measurements)
        # 写入文件（每条都记）
        if self._rtt_debug_file is not None:
            try:
                self._rtt_debug_file.write(f"{seq},{rtt_ms:.3f}\n")
            except Exception:
                pass
        # 定期打印统计
        if seq % self._rtt_log_interval == 0:
            recent = self._rtt_measurements[-self._rtt_log_interval:]
            arr = np.array(recent)
            self.get_logger().info(
                f'📡 网络RTT统计 (最近{self._rtt_log_interval}条): '
                f'均值={np.mean(arr):.2f}ms, '
                f'中位={np.median(arr):.2f}ms, '
                f'最大={np.max(arr):.2f}ms, '
                f'最小={np.min(arr):.2f}ms, '
                f'标准差={np.std(arr):.2f}ms'
            )

    # ============================================================
    # 坐标映射：位置
    # ============================================================

    def _map_device_to_robot_position(self, dev_pos):
        delta_device = dev_pos - self._device_origin_pos # 主手相对锚点的增量
        delta_robot = self.R_DEV2ROB @ (delta_device * self._position_scale) # 缩放后转到robot系
        target_pos = self._robot_base_pos + delta_robot # 叠加到机器人基准位置上

        alpha = self._smoothing_alpha
        self._smoothed_pos = alpha * target_pos + (1.0 - alpha) * self._smoothed_pos

        return self._smoothed_pos.copy()

    # ============================================================
    # 坐标映射：姿态
    # ============================================================

    def _map_device_to_robot_orientation(self, dev_quat_wxyz):
        if not self._enable_ori_tracking:
            return self._current_target_quat_xyzw.copy()

        q_anchor_inv = quat_conjugate(self._device_origin_quat_wxyz)
        q_delta_dev = quat_multiply(dev_quat_wxyz, q_anchor_inv)
        q_delta_dev = quat_normalize(q_delta_dev)

        q_map = self.Q_DEV2ROB_WXYZ
        q_map_conj = quat_conjugate(q_map)
        q_delta_rob = quat_multiply(quat_multiply(q_map, q_delta_dev), q_map_conj)
        q_delta_rob = quat_normalize(q_delta_rob)

        if abs(self._orientation_scale - 1.0) > 0.01:
            identity = np.array([1.0, 0.0, 0.0, 0.0])
            q_delta_rob = quat_normalize(
                (1.0 - self._orientation_scale) * identity +
                self._orientation_scale * q_delta_rob
            )

        q_target_wxyz = quat_multiply(q_delta_rob, self._robot_base_quat_wxyz)
        q_target_wxyz = quat_normalize(q_target_wxyz)

        target_quat_xyzw = np.array([
            q_target_wxyz[1], q_target_wxyz[2],
            q_target_wxyz[3], q_target_wxyz[0],
        ])

        return target_quat_xyzw

    # ============================================================
    # 定时器回调（100Hz）
    # ============================================================

    def _timer_callback(self):

        state = self._wrapper.get_state()

        self._wrapper.set_external_force(self._external_force_device)

        current_buttons = state.buttons
        pressed = [False, False, False]
        released = [False, False, False]
        for i in range(3):
            if current_buttons[i] and not self._prev_buttons[i]:
                pressed[i] = True
            if not current_buttons[i] and self._prev_buttons[i]:
                released[i] = True
        self._prev_buttons = list(current_buttons)

        if pressed[ButtonID.B]:
            self._enabled = True
            self._device_origin_pos = state.position.copy()
            self._device_origin_quat_wxyz = state.orientation_wxyz.copy()
            self._smoothed_pos = self._robot_base_pos.copy()
            
            self.get_logger().info(
                f'遥操作使能 | 基准pos={self._robot_base_pos}'
            )
            # ★ 重置阻抗 + 接触检测 + 变刚度状态
            self._filtered_impedance_force = np.zeros(3)
            self._actual_robot_vel = np.zeros(3)
            self._prev_actual_time = 0.0
            self._prev_pos_error = np.zeros(3)
            self._filtered_d_error = np.zeros(3)        # ★ NEW: d_error 独立滤波
            self._contact_state = 'free'
            self._pulse_start_time = 0.0
            self._pulse_direction = np.zeros(3)
            self._prev_sensor_force_mag = 0.0
            self._impedance_frame_count = 0
            self._tracking_delta_actual.clear()
            self._tracking_delta_target.clear()
            self._prev_actual_for_tracking = self._actual_robot_pos.copy()
            self._prev_target_for_tracking = self._current_target_pos.copy()
            self._current_tracking_ratio = 1.0

        if released[ButtonID.B]:
            self._robot_base_pos = self._smoothed_pos.copy()
            if self._enable_ori_tracking:
                xyzw = self._current_target_quat_xyzw
                self._robot_base_quat_wxyz = np.array([
                    xyzw[3], xyzw[0], xyzw[1], xyzw[2]
                ])
            self._enabled = False
            self.get_logger().info(
                f'遥操作禁用 | 冻结pos={self._robot_base_pos}'
            )
            # ★ 禁用时清零所有力 + 接触状态
            self._filtered_impedance_force = np.zeros(3)
            self._filtered_d_error = np.zeros(3)    # ★ NEW
            self._external_force_device = [0.0, 0.0, 0.0]
            self._wrapper.set_external_force([0.0, 0.0, 0.0])
            self._contact_state = 'free'
            self._pulse_start_time = 0.0
            self._pulse_direction = np.zeros(3)
            self._impedance_frame_count = 0

        # ---- 按钮 A：夹爪切换 ----
        if pressed[ButtonID.A]:
            # ★★★ 切换夹爪状态 ★★★
            self._gripper_closed = not self._gripper_closed
            gripper_cmd = 1 if self._gripper_closed else 0
            
            # 发布夹爪命令
            msg = Int32()
            msg.data = gripper_cmd
            self._pub_gripper.publish(msg)
            
            # 打印日志
            status_str = "🔴 闭合" if self._gripper_closed else "🟢 打开"
            self.get_logger().info(f'夹爪切换 → {status_str} (命令: {gripper_cmd})')

        if self._enabled:
            self._current_target_pos = self._map_device_to_robot_position(
                state.position
            )
            self._current_target_quat_xyzw = self._map_device_to_robot_orientation(
                state.orientation_wxyz
            )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base'

        pose_msg.pose.position.x = float(self._current_target_pos[0])
        pose_msg.pose.position.y = float(self._current_target_pos[1])
        pose_msg.pose.position.z = float(self._current_target_pos[2])

        pose_msg.pose.orientation.x = float(self._current_target_quat_xyzw[0])
        pose_msg.pose.orientation.y = float(self._current_target_quat_xyzw[1])
        pose_msg.pose.orientation.z = float(self._current_target_quat_xyzw[2])
        pose_msg.pose.orientation.w = float(self._current_target_quat_xyzw[3])

        self._pub_pose.publish(pose_msg)
        # ★ NEW: 发布 Z 轴期望力
        # 发送频率 = 定时器频率 / divider = 100Hz / 5 = 20Hz
        # 控制器端超时阈值已改为 5.0s，所以 20Hz 绰绰有余
        # 只有在遥操作使能时才发送力（防止静止时也施加力）
        if self._enable_force_z and self._loop_count % self._force_z_divider == 0:
            force_msg = Float64()
            # 负值 = 向下推（Franka base 系 Z 轴向上，所以 -3.0 表示向下施加 3N）
            # 如果不希望遥操作禁用时也施力，可以加条件判断
            if self._enabled:
                # force_msg.data = float(self._force_z_desired)
                force_msg.data = float(self.get_parameter('force_z_desired').value)
            else:
                force_msg.data = 0.0  # 禁用时不施力
            self._pub_force_z.publish(force_msg)
            self._force_z_pub_count += 1
        # ★★★ 降低夹爪发送频率：每20次发送一次（2Hz）★★★
        if self._loop_count % 20 == 0:
            gripper_msg = Int32()
            gripper_msg.data = 1 if self._gripper_closed else 0
            self._pub_gripper.publish(gripper_msg)
        state_msg = DeviceStateMsg()

        state_msg = DeviceStateMsg()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'device'
        state_msg.position = state.position.tolist()
        state_msg.velocity = state.velocity.tolist()
        state_msg.orientation_xyzw = state.orientation_xyzw.tolist()
        state_msg.orientation_wxyz = state.orientation_wxyz.tolist()
        state_msg.buttons = state.buttons
        state_msg.is_connected = state.is_connected
        state_msg.data_rate_hz = state.data_rate_hz
        self._pub_state.publish(state_msg)

        # ---- 状态打印（每秒一次）----
        self._loop_count += 1
        if self._loop_count % 100 == 0:
            e = '🟢使能' if self._enabled else '🔴冻结'
            t = self._current_target_pos
            raw_f = self._raw_force_robot  # 显示原始力
            f = self._filtered_force_robot
            f_mag = np.linalg.norm(f)
            f_dev = self._external_force_device

            # ★ CHANGED: 增加阻抗状态打印
            if self._force_callback_count > 0:
                # 基础力信息
                force_str = (
                    f'F_sen=[{raw_f[0]:+.1f},{raw_f[1]:+.1f},{raw_f[2]:+.1f}] '
                    f'F_dev=[{f_dev[0]:+.2f},{f_dev[1]:+.2f},{f_dev[2]:+.2f}]'
                )
                if self._actual_pos_received:
                    pos_err = self._current_target_pos - self._actual_robot_pos
                    err_mag = np.linalg.norm(pos_err) * 1000
                    imp_f = self._filtered_impedance_force
                    tr = self._current_tracking_ratio
                    cs = '🔴触' if self._contact_state == 'contact' else '⚪空'
                    imp_str = (
                        f'Δ={err_mag:.1f}mm '
                        f'tr={tr:.2f} {cs} '
                        f'Fi=[{imp_f[0]:+.2f},{imp_f[1]:+.2f},{imp_f[2]:+.2f}]'
                    )
                else:
                    imp_str = '⚠️未收到实际位置'
                force_status = f'{force_str} | {imp_str} | cb#{self._force_callback_count}'
                # 三元表达式写法（更简洁）
                # result = value_if_true if condition else value_if_false
            else:
                force_status = f'⚠️ 尚未收到力数据 (回调0次)'
            self.get_logger().info(
                f'{e} | target=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] | '
                f'grip={"🔴" if self._gripper_closed else "🟢"} | '  # ★ 新增
                f'{force_status}'
            )
            # ===== 改为（在 grip 后面加一段 Fz 信息）=====
            fz_str = f'Fz={self._force_z_desired:.1f}N' if self._enable_force_z else 'Fz=OFF'
            self.get_logger().info(
                f'{e} | target=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] | '
                f'grip={"🔴" if self._gripper_closed else "🟢"} | '
                f'{fz_str}(#{self._force_z_pub_count}) | '
                f'{force_status}'
            )

    # ============================================================
    # 清理
    # ============================================================

    def destroy_node(self):
        self.get_logger().info('正在停止设备...')

        if self._force_debug_file is not None:
            try:
                self._force_debug_file.write(
                    f"\n# 结束 - 总力回调次数: {self._force_callback_count}\n"
                    f"# 总调试记录次数: {self._force_debug_counter}\n"
                )
                self._force_debug_file.close()
                self.get_logger().info(
                    f'调试文件已关闭，共记录 {self._force_debug_counter} 条'
                )
            except Exception:
                pass

        # 关闭 RTT 日志
        if self._rtt_debug_file is not None:
            try:
                self._rtt_debug_file.close()
                self.get_logger().info(
                    f'RTT日志已关闭，共 {len(self._rtt_measurements)} 条'
                )
            except Exception:
                pass

        self._wrapper.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeviceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()


# # 真实的ROS2消息内容
# header:
#   stamp:
#     sec: 1234567890
#     nanosec: 123456789
#   frame_id: "base"  # 或"world"等坐标系
# wrench:
#   force:
#     x: 1.5           # 浮点数，单位牛顿(N)
#     y: -0.8          # 浮点数，单位牛顿(N)  
#     z: 9.2           # 浮点数，单位牛顿(N)
#   torque:
#     x: 0.0           # 目前代码中不使用
#     y: 0.0           # 目前代码中不使用
#     z: 0.0           # 目前代码中不使用

# 力的发布赫兹：average rate: 98.526 /end_effector_force min: 0.000s max: 0.099s std dev: 0.00844s window: 893
# 测延迟：ta@ta-ThinkBook-14-G8-IAL:~/ros2_ws2/src/teleop_nodes/teleop_nodes$ ros2 topic delay /device/pose 
# average delay: 0.001
# 	min: 0.000s max: 0.001s std dev: 0.00022s window: 199
# 测位置发布端频率：ros2 topic hz /device/pose
# average rate: 200.026基本稳定
# 	min: 0.004s max: 0.006s std dev: 0.00029s window: 202

# 这个没测量： ros2 topic hz /device/pose
# 请提供禁用自动GC相关，目前已经添加了import gc; gc.disable()但效果一般还是有卡顿
# 让定时器回调和力反馈回调在不同线程里并行执行，互不阻塞。main() 函数里 rclpy.spin(node) 改为用 MultiThreadedExecutor，并把力反馈回调放进 ReentrantCallbackGroup
# /device/pose 改用 BEST_EFFORT
# ping -i 0.005 -c 1000 10.150.11.81 测试结果最大16ms，最小5ms

#📋 消息类型定义位置:
# 1️⃣ DeviceStateMsg
# 位置：/home/ta/ros2_ws2/src/teleop_msgs/msg/DeviceState.msg