import threading
import signal
from typing import List
import numpy as np
import time
import asyncio
import websockets
import orjson
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from force_controller_1 import ForceFeedbackCalculator, IS_FORCE_FEEDBACK_ENABLED
from config_manager import ConfigMode, get_value


'''
设备通信模块
实现inverse主端数据封装
建立与设备的WebSocket连接
接收设备状态数据（位置、速度、按钮状态、方向等）
发送力反馈命令到设备
实现连接重试和错误处理
提供设备状态查询接口
'''

class Device:
    def __init__(self, config_mode: ConfigMode = ConfigMode.LEADER):
        self.config_mode = config_mode
        self._load_config_parameters()
        
        # 初始化力反馈计算器
        self.force_calculator = ForceFeedbackCalculator(config_mode=config_mode)
        
        # 设备状态
        self.force = [0, 0, 0]
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.buttons = [False, False, False]
        self.base_orientation = [0, 0, 1, 0] # Quaternion(x, y, z, w)
        self.tool_orientation = [0, 0, 0, 0] # Quaternion(x, y, z, w)
        self._running = True
        self._lock = threading.Lock()

        # 外部力反馈（如来自 MuJoCo 的接触力），由外部线程写入
        self._external_force = [0.0, 0.0, 0.0]

        # 性能统计
        self.data_refresh_rate = 0.0  
        self.msg_count_since_last = 0
        self.last_data_update_time = time.perf_counter()  
        self.last_rate_check_time = self.last_data_update_time

        
        print(f"设备初始化完成，配置模式: {config_mode.value}")
        print(f"URI: {self.uri}, 坐标原点: {self.coordinate_origin}, 坐标基: {self.basis}")




    def _load_config_parameters(self):
        # 从配置文件读取参数，这些配置文件位于项目的config目录下，主端leader.toml，从端follower.toml
        self.uri = get_value("network.uri", "ws://localhost:10001", self.config_mode)
        self.coordinate_origin = get_value("coordinate.origin", "workspace_center", self.config_mode)
        self.basis = get_value("coordinate.basis", "XYZ", self.config_mode)
        self.max_retries = get_value("network.max_retries", 3, self.config_mode)
        self.timeout = get_value("network.timeout", 5.0, self.config_mode)
        self.reconnect_interval = get_value("network.reconnect_interval", 1.0, self.config_mode)
        self.force_feedback_enabled = get_value("device.force_feedback_enabled", True, self.config_mode)

    async def main(self):
        '''
        Main asynchronous loop that connects to the server and receives data from the devices.
        '''
        first_message = True
        inverse3_device_id = None
        retry_count = 0
        
        # 主循环：在运行状态且未超过最大重试次数时持续尝试连接
        while self._running and retry_count < self.max_retries:
            try:
                # 建立WebSocket连接
                async with websockets.connect(self.uri) as ws: # 异步 WebSocket 连接，返回一个异步上下文对象
                    # as ws 把建立成功的 WebSocket 连接实例绑定到变量 ws，在 async with 块内，你可以通过 ws 执行异步的收发操作
                    '''
                    进入 async with 块时，会执行异步资源的 “获取” 逻辑（如建立 WebSocket 连接）；
                    退出 async with 块时（无论正常结束 / 异常），会自动执行异步资源的 “释放” 逻辑（如关闭 WebSocket 连接）；
                    注意：async with 必须在异步函数（async def 定义的函数） 中使用，否则会报错。
                    '''
                    # 数据接收循环：在运行状态下持续处理数据
                    while self._running:
                        try:
                            data = orjson.loads(await asyncio.wait_for(ws.recv(), timeout=self.timeout)) # 接收并解析JSON数据（使用orjson提高性能）

                            # 记录当前时间并更新消息计数
                            current_time = time.perf_counter()
                            self.msg_count_since_last += 1

                            if current_time - self.last_rate_check_time >= 1.0: # 超时机制防止堵塞
                                self.data_refresh_rate = self.msg_count_since_last / (current_time - self.last_rate_check_time) # 计算数据刷新率
                                self.msg_count_since_last = 0
                                self.last_rate_check_time = current_time
                            
                            # 从接收的数据中解析设备信息
                            inverse3_devices = data.get("inverse3", [])           # Inverse3设备列表
                            '''
                            data.get(key, default) 是字典的核心方法，作用是安全获取指定键的值：
                            如果 data 中存在键 "inverse3"，则返回该键对应的值（通常是列表）；
                            如果不存在该键，或该键对应的值为 None，则返回默认值 []（空列表）
                            '''
                            verse_grip_devices = data.get("wireless_verse_grip", [])  # VerseGrip手柄列表
                            # 提取第一个设备的数据（如果存在）
                            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
                            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

                            # 第一次消息时打印设备信息
                            if first_message:
                                first_message = False
                                if not inverse3_data:
                                    print("Error: No Inverse3 device found.")
                                    retry_count += 1
                                    break
                                else:
                                    inverse3_device_id = inverse3_data.get("device_id")
                                    base_ori = inverse3_data["state"].get("body_orientation", {})
                                    self.base_orientation = [base_ori["x"], base_ori["y"], base_ori["z"], base_ori["w"]]
                                    request_msg = {"inverse3": [{"device_id": inverse3_device_id, "commands": {}}]}
                                    request_msg["inverse3"][0]["commands"]["set_coordinate_origin"] = {"coordinate_origin": self.coordinate_origin}
                                    request_msg["inverse3"][0]["commands"]["set_basis"] = {"basis": {"permutation": self.basis}}
                                    print(f"Inverse3 ID: {inverse3_device_id}, base orientation: {self.base_orientation}")

                                if not verse_grip_data:
                                    print("Warning: No VerseGrip found.")
                                else:
                                    verse_grip_id = verse_grip_data.get("device_id")
                                    battery_level = verse_grip_data["state"].get("battery_level")
                                    print(f"VerseGrip ID: {verse_grip_id}, battery level: {battery_level*1e2:.1f}%")

                            # 提取设备状态数据
                            pos = inverse3_data["state"].get("cursor_position", {})    # 光标位置
                            vel = inverse3_data["state"].get("cursor_velocity", {})    # 光标速度
                            btn = verse_grip_data["state"].get("buttons", {})           # 按钮状态
                            tool_ori = verse_grip_data["state"].get("orientation", {}) # 工具方向
 
                            # 使用线程锁更新数据
                            with self._lock:
                                self.position = list(pos.values())
                                self.velocity = list(vel.values())
                                self.buttons = list(btn.values())
                                self.tool_orientation = list(tool_ori.values())
                            
                            # 记录数据更新时间戳
                            self.last_data_update_time = current_time
                            # 计算力反馈（如果启用）或使用零力
                            # calculated_force = self.force_calculator.calculate_total_force(self.position, self.velocity) if self.force_feedback_enabled else [0, 0, 0]
                            
                            # ★ 读取外部力（加锁）
                            with self._lock:
                                ext_force = self._external_force.copy()
                            # ★ 将外部力传入力计算器（叠加到墙力/地板力之上）
                            calculated_force = self.force_calculator.calculate_total_force(
                                self.position, self.velocity,
                                external_force=ext_force
                            ) if self.force_feedback_enabled else [0, 0, 0]

                            self.force = calculated_force
                            
                            # 构建力反馈命令
                            frc = {"x": float(calculated_force[0]), "y": float(calculated_force[1]), "z": float(calculated_force[2])}   
                            # 将力反馈命令添加到请求消息                         
                            request_msg["inverse3"][0]["commands"]["set_cursor_force"] = {"values": frc}

                            # 异步发送消息（await 必须加，因为是异步操作）
                            await ws.send(orjson.dumps(request_msg))
  
                        # 处理连接关闭错误（自动重连）
                        except asyncio.TimeoutError:
                            print("Warning: Device communication timeout, retrying...")
                            retry_count += 1
                            break
                        except Exception as e:
                            print(f"Error processing device data: {str(e)}")
                            retry_count += 1
                            break

            except websockets.exceptions.ConnectionClosed:
                print("Connection closed unexpectedly, reconnecting...")
                retry_count += 1

                # 异步接收消息
                await asyncio.sleep(self.reconnect_interval)
            except Exception as e:
                print(f"Connection error: {str(e)}")
                retry_count += 1
                await asyncio.sleep(self.reconnect_interval)

        if retry_count >= self.max_retries: # 如果重试次数达到最大值，则停止重连
            print("Error: Maximum connection retries reached, exiting..")
            self._running = False

    # 数据获取接口，向外界提供的是Inverse主手设备的实时状态数据
    # Inverse3和VerseGrip设备通过WebSocket发送状态数据
    def get_force(self) -> List[float]: # List[float] 表示返回值是一个列表，且列表中的元素都是浮点数
        with self._lock:
            return self.force.copy() # self.force 是列表对象，copy() 方法会创建列表的浅拷贝，返回一个新的列表对象
        '''
        with 是 Python 的上下文管理器语法，
        核心作用是：自动管理资源（如锁、文件、网络连接），进入 with 块时执行资源的 “获取” 操作，退出时自动执行 “释放” 操作

        with self._lock 等价于：
        self._lock.acquire()  # 获取锁（加锁）
        try:
            return self.force.copy()
        finally:
            self._lock.release()  # 无论是否异常，都释放锁，自己拿锁自己放锁
        '''
    
    def get_position(self) -> List[float]:
        with self._lock:
            return self.position.copy()
    
    def get_velocity(self) -> List[float]:
        with self._lock:
            return self.velocity.copy()
    
    def get_buttons(self) -> List[bool]:
        with self._lock:
            return self.buttons.copy()
    
    def get_orientation(self) -> List[float]:
        with self._lock:
            return self.tool_orientation.copy()
    
    def get_last_data_update_time(self) -> float:
        return self.last_data_update_time
    
    def update_force_feedback_parameters(self, **kwargs):
        self.force_calculator.update_parameters(**kwargs)
    
    def get_force_feedback_parameters(self) -> dict:
        return self.force_calculator.get_parameters()
    
    def get_performance_stats(self) -> dict:
        return {
            'data_refresh_rate_hz': self.data_refresh_rate,
            'last_data_update_time': self.last_data_update_time
        }
    
    def set_external_force(self, force: list):
        """
        设置外部力（线程安全）。
        该力会在下次力计算循环中叠加到总力输出。
        Args:
            force: [fx, fy, fz] 设备坐标系，牛顿
        """
        with self._lock:
            self._external_force = [float(force[0]), float(force[1]), float(force[2])]

    def stop(self):
        print("正在停止设备通信...")
        self._running = False

        time.sleep(0.1)
        print("设备通信已停止")


    def run_inverse(self):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.main())
        except asyncio.CancelledError:
            print("异步任务被取消")
        except Exception as e:
            print(f"异步循环错误: {e}")
        finally:
            if loop.is_running():
                loop.stop()
            loop.close()

def signal_handler(signum, frame):
    print(f"\n接收到信号 {signum}，正在关闭...")
    raise KeyboardInterrupt

'''
设备 → WebSocket：Inverse3和VerseGrip设备通过WebSocket发送状态数据
WebSocket → 解析：main()函数接收并解析数据
解析 → 存储：使用线程锁将数据存储到实例变量
存储 → 接口：通过get_*函数提供外部访问
'''

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)   
    signal.signal(signal.SIGTERM, signal_handler)  
    
    dev = Device()
    thread = None
    
    try:
        print("启动设备通信线程...")
        # 在__main__中创建子线程
        thread = threading.Thread(target=dev.run_inverse, args=(), daemon=True)
        thread.start()
        
        print("设备通信已启动，按 Ctrl+C 退出")
        print("=" * 50)
        
        while thread.is_alive():
            try:
                time.sleep(1)
                if thread.is_alive():
                    print(f"位置: {dev.get_position()}")
                    pass
                    
            except KeyboardInterrupt:
                break
                
    except KeyboardInterrupt:
        print("\n程序被中断，开始关闭...")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        print("\n" + "=" * 50)
        print("开始清理资源...")

        dev.stop()
        
        if thread and thread.is_alive():
            print("等待通信线程结束...")
            thread.join(timeout=5.0)  # 等待5秒

        print("程序退出")
