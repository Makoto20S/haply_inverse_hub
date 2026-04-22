"""
device_wrapper.py
对 Inverse3 + VerseGrip 设备数据的高层封装
提供带时间戳、坐标转换、按钮边沿检测的干净接口
下游（MuJoCo 控制器等）只需要和这个文件打交道
"""

import threading
import time
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from enum import IntEnum

# ---- 导入现有模块 ----
from inverse_comm import Device
from config_manager import ConfigMode
from transformation import quat_to_axes_xyzw, fix_quat_continuity


# ============================================================
# 数据结构定义
# ============================================================

class ButtonID(IntEnum):
    """
    VerseGrip 按钮编号
    对应硬件：
      A = 按钮②（输入按钮，食指侧）
      B = 按钮③（输入按钮，拇指侧）
      C = 按钮⑤（校准按钮，笔尾部）
    """
    A = 0
    B = 1
    C = 2


@dataclass
class DeviceState:
    """
    设备状态快照 —— 单次采样的所有数据打包在一起
    
    所有数据都带时间戳，下游可以判断数据新鲜度。
    四元数同时提供 xyzw（设备原始）和 wxyz（MuJoCo 用）两种格式。
    """
    # ---- 时间戳 ----
    timestamp: float = 0.0                # time.perf_counter() 秒

    # ---- Inverse3 数据 ----
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))     # 末端位置 [x,y,z] 米
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))     # 末端速度 [vx,vy,vz] 米/秒
    force: np.ndarray = field(default_factory=lambda: np.zeros(3))        # 当前力反馈 [fx,fy,fz] 牛顿

    # ---- VerseGrip 数据 ----
    orientation_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))  # 四元数 [x,y,z,w] 设备原始格式
    orientation_wxyz: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # 四元数 [w,x,y,z] MuJoCo 格式

    # ---- 按钮状态 ----
    buttons: List[bool] = field(default_factory=lambda: [False, False, False])  # [A, B, C] 当前状态
    buttons_pressed: List[bool] = field(default_factory=lambda: [False, False, False])   # 本次刚按下（上升沿）
    buttons_released: List[bool] = field(default_factory=lambda: [False, False, False])  # 本次刚松开（下降沿）

    # ---- 连接状态 ----
    is_connected: bool = False            # 设备是否在线
    data_rate_hz: float = 0.0             # 实际数据刷新率


# ============================================================
# 设备封装器
# ============================================================

class DeviceWrapper:
    """
    Inverse3 + VerseGrip 设备的高层封装
    
    使用方法：
        wrapper = DeviceWrapper()
        wrapper.start()          # 启动通信（内部自动起线程）
        
        state = wrapper.get_state()   # 随时获取最新状态
        print(state.position)
        print(state.buttons_pressed)
        
        wrapper.stop()           # 停止通信
    
    也可以用 with 语句自动管理生命周期：
        with DeviceWrapper() as wrapper:
            state = wrapper.get_state()
    """

    def __init__(self, config_mode: ConfigMode = ConfigMode.LEADER, 
                 startup_wait: float = 2.0,
                 stale_threshold: float = 0.5):
        """
        Args:
            config_mode: 配置模式，LEADER=主手真机，FOLLOWER=从端仿真
            startup_wait: start() 后等待连接建立的秒数
            stale_threshold: 数据超过多少秒没更新就判定为掉线
        """
        self._config_mode = config_mode
        self._startup_wait = startup_wait
        self._stale_threshold = stale_threshold

        # 底层设备实例（来自 inverse_comm.py）
        self._device: Optional[Device] = None
        self._comm_thread: Optional[threading.Thread] = None
        self._running = False

        # 按钮边沿检测用的上一帧状态
        self._prev_buttons: List[bool] = [False, False, False]

        # 四元数连续性处理
        self._prev_quat: Optional[np.ndarray] = None
        self._quat_jump_count: int = 0

        # 线程锁（保护 _prev_buttons 和 _prev_quat）
        self._lock = threading.Lock()

    # ---- 生命周期管理 ----

    def start(self) -> bool:
        """
        启动设备通信
        
        Returns:
            True=连接成功，False=超时未连接
        """
        if self._running:
            print("[DeviceWrapper] 已经在运行，跳过重复启动")
            return True

        print(f"[DeviceWrapper] 初始化设备，模式: {self._config_mode.value}")
        self._device = Device(config_mode=self._config_mode)

        print("[DeviceWrapper] 启动通信线程...")
        self._comm_thread = threading.Thread(
            target=self._device.run_inverse,
            daemon=True,  # 守护线程，主程序退出时自动结束
            name="inverse_comm"
        )
        self._comm_thread.start()
        self._running = True

        # 等待连接建立
        print(f"[DeviceWrapper] 等待设备连接（最多 {self._startup_wait} 秒）...")
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < self._startup_wait:
            if self._device.get_last_data_update_time() > start_time:
                elapsed = time.perf_counter() - start_time
                print(f"[DeviceWrapper] 设备连接成功（耗时 {elapsed:.2f} 秒）")
                return True
            time.sleep(0.05)

        # 超时检查：可能连上了但时间戳没更新
        if self._device.get_last_data_update_time() > 0:
            print("[DeviceWrapper] 设备连接成功（在等待期间）")
            return True
        else:
            print("[DeviceWrapper] 警告：等待超时，设备可能未连接，但通信线程仍在运行")
            return False

    def stop(self):
        """停止设备通信，释放资源"""
        if not self._running:
            return

        print("[DeviceWrapper] 正在停止...")
        self._running = False

        if self._device:
            self._device.stop()

        if self._comm_thread and self._comm_thread.is_alive():
            self._comm_thread.join(timeout=5.0)
            if self._comm_thread.is_alive():
                print("[DeviceWrapper] 警告：通信线程未在 5 秒内结束")

        print("[DeviceWrapper] 已停止")

    def __enter__(self):
        """支持 with 语句"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持 with 语句"""
        self.stop()
        return False  # 不吞异常

    # ---- 核心数据接口 ----

    def get_state(self) -> DeviceState:
        """
        获取当前设备状态快照
        
        每次调用都会：
        1. 从底层 Device 读取原始数据
        2. 转换四元数格式（xyzw → wxyz）
        3. 检测按钮边沿（刚按下/刚松开）
        4. 处理四元数连续性
        5. 判断连接状态
        
        Returns:
            DeviceState 数据结构，所有字段都已填充
        """
        if self._device is None:
            return DeviceState()  # 返回全零默认值

        state = DeviceState()
        now = time.perf_counter()
        state.timestamp = now

        # ---- 读取原始数据 ----
        raw_pos = self._device.get_position()       # [x, y, z]
        raw_vel = self._device.get_velocity()       # [vx, vy, vz]
        raw_force = self._device.get_force()        # [fx, fy, fz]
        raw_buttons = self._device.get_buttons()    # [a, b, c]
        raw_ori = self._device.get_orientation()    # [x, y, z, w]

        # ---- 位置、速度、力 ----
        state.position = np.array(raw_pos, dtype=np.float64)
        state.velocity = np.array(raw_vel, dtype=np.float64)
        state.force = np.array(raw_force, dtype=np.float64)

        # ---- 四元数处理 ----
        quat_xyzw = np.array(raw_ori, dtype=np.float64)

        # 归一化（防止数值漂移）
        quat_norm = np.linalg.norm(quat_xyzw)
        if quat_norm > 1e-8:
            quat_xyzw = quat_xyzw / quat_norm

        # 连续性处理（防止符号翻转导致的跳变）
        with self._lock:
            quat_xyzw, self._quat_jump_count = fix_quat_continuity(
                quat_xyzw, self._prev_quat, self._quat_jump_count
            )
            self._prev_quat = quat_xyzw.copy()

        state.orientation_xyzw = quat_xyzw
        # 转换为 MuJoCo 格式 [w, x, y, z]
        state.orientation_wxyz = np.array([
            quat_xyzw[3],  # w
            quat_xyzw[0],  # x
            quat_xyzw[1],  # y
            quat_xyzw[2],  # z
        ], dtype=np.float64)

        # ---- 按钮边沿检测 ----
        # 确保 raw_buttons 有 3 个元素
        current_buttons = [False, False, False]
        for i in range(min(len(raw_buttons), 3)):
            current_buttons[i] = bool(raw_buttons[i])

        with self._lock:
            state.buttons = current_buttons.copy()
            state.buttons_pressed = [
                current_buttons[i] and not self._prev_buttons[i]
                for i in range(3)
            ]
            state.buttons_released = [
                not current_buttons[i] and self._prev_buttons[i]
                for i in range(3)
            ]
            self._prev_buttons = current_buttons.copy()

        # ---- 连接状态 ----
        last_update = self._device.get_last_data_update_time()
        state.is_connected = (now - last_update) < self._stale_threshold if last_update > 0 else False

        stats = self._device.get_performance_stats()
        state.data_rate_hz = stats.get('data_refresh_rate_hz', 0.0)

        return state

    # ---- 便捷方法（不想每次都拿完整 state 时用） ----

    def get_position(self) -> np.ndarray:
        """获取末端位置 [x,y,z] 米"""
        if self._device is None:
            return np.zeros(3)
        return np.array(self._device.get_position(), dtype=np.float64)

    def get_velocity(self) -> np.ndarray:
        """获取末端速度 [vx,vy,vz] 米/秒"""
        if self._device is None:
            return np.zeros(3)
        return np.array(self._device.get_velocity(), dtype=np.float64)

    def get_orientation_wxyz(self) -> np.ndarray:
        """获取四元数 [w,x,y,z]（MuJoCo 格式）"""
        state = self.get_state()
        return state.orientation_wxyz

    def get_orientation_xyzw(self) -> np.ndarray:
        """获取四元数 [x,y,z,w]（设备原始格式 / scipy 格式）"""
        state = self.get_state()
        return state.orientation_xyzw

    def is_button_held(self, button: ButtonID) -> bool:
        """按钮是否正在被按住"""
        if self._device is None:
            return False
        buttons = self._device.get_buttons()
        if button < len(buttons):
            return bool(buttons[button])
        return False

    def is_connected(self) -> bool:
        """设备是否在线"""
        if self._device is None:
            return False
        last_update = self._device.get_last_data_update_time()
        return (time.perf_counter() - last_update) < self._stale_threshold if last_update > 0 else False

    # ---- 力反馈控制 ----

    def update_force_params(self, **kwargs):
        """
        更新力反馈参数
        
        示例：
            wrapper.update_force_params(wall_stiffness=800.0, force_limit=6.0)
        """
        if self._device:
            self._device.update_force_feedback_parameters(**kwargs)

    def get_force_params(self) -> dict:
        """获取当前力反馈参数"""
        if self._device:
            return self._device.get_force_feedback_parameters()
        return {}
    
    def set_external_force(self, force: list):
        """
        设置外部力反馈（如 MuJoCo 接触力），会叠加到力计算器的输出上。
        Args:
            force: [fx, fy, fz] 设备坐标系下的力，单位：牛顿
                   传 [0,0,0] 表示无外部力
        """
        if self._device is not None:
            self._device.set_external_force(force)

    # ---- 调试工具 ----

    def print_state(self, state: Optional[DeviceState] = None):
        """打印设备状态（调试用）"""
        if state is None:
            state = self.get_state()

        conn_str = "✅ 在线" if state.is_connected else "❌ 离线"
        print(f"\n{'='*65}")
        print(f"  设备状态 [{conn_str}]  刷新率: {state.data_rate_hz:.0f} Hz")
        print(f"{'='*65}")
        print(f"  位置 (mm) : [{state.position[0]*1e3:+8.2f}, {state.position[1]*1e3:+8.2f}, {state.position[2]*1e3:+8.2f}]")
        print(f"  速度 (mm/s): [{state.velocity[0]*1e3:+8.2f}, {state.velocity[1]*1e3:+8.2f}, {state.velocity[2]*1e3:+8.2f}]")
        print(f"  力   (N)  : [{state.force[0]:+6.2f}, {state.force[1]:+6.2f}, {state.force[2]:+6.2f}]")
        print(f"  姿态 xyzw : [{state.orientation_xyzw[0]:+.4f}, {state.orientation_xyzw[1]:+.4f}, {state.orientation_xyzw[2]:+.4f}, {state.orientation_xyzw[3]:+.4f}]")
        print(f"  姿态 wxyz : [{state.orientation_wxyz[0]:+.4f}, {state.orientation_wxyz[1]:+.4f}, {state.orientation_wxyz[2]:+.4f}, {state.orientation_wxyz[3]:+.4f}]")
        print(f"  按钮 A/B/C: {state.buttons}  按下: {state.buttons_pressed}  松开: {state.buttons_released}")
        print(f"{'='*65}")


# ============================================================
# 测试入口
# ============================================================

if __name__ == "__main__":
    import signal

    def signal_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    wrapper = DeviceWrapper(config_mode=ConfigMode.LEADER)

    try:
        connected = wrapper.start()
        if not connected:
            print("设备未连接，但仍尝试运行...")

        print("\n按 Ctrl+C 退出")
        print("请操作 Inverse3 和 VerseGrip，观察数据变化\n")

        loop_count = 0
        while True:
            state = wrapper.get_state()

            # 每 20 次循环打印一次完整状态
            if loop_count % 20 == 0:
                wrapper.print_state(state)

            # 实时打印按钮事件（只在按下/松开瞬间打印）
            for i, name in enumerate(["A", "B", "C"]):
                if state.buttons_pressed[i]:
                    print(f"  >>> 按钮 {name} 按下！")
                if state.buttons_released[i]:
                    print(f"  <<< 按钮 {name} 松开！")

            loop_count += 1
            time.sleep(0.1)  # 10 Hz 打印频率

    except KeyboardInterrupt:
        print("\n中断，正在退出...")
    finally:
        wrapper.stop()
        print("程序结束")


'''
核心数据结构是 DeviceState，包含以下数据：

数据类型	字段名	说明	单位
时间戳	timestamp	采样时间	秒
位置	position	Inverse3末端位置	米 [x,y,z]
速度	velocity	Inverse3末端速度	米/秒 [vx,vy,vz]
力反馈	force	当前力反馈	牛顿 [fx,fy,fz]
姿态(设备格式)	orientation_xyzw	VerseGrip四元数	[x,y,z,w]
姿态(MuJoCo格式)	orientation_wxyz	转换后的四元数	[w,x,y,z]
按钮状态	buttons	A/B/C当前状态	布尔列表
按钮上升沿	buttons_pressed	刚按下	布尔列表
按钮下降沿	buttons_released	刚松开	布尔列表
连接状态	is_connected	设备是否在线	布尔
数据刷新率	data_rate_hz	实际刷新率	Hz
'''