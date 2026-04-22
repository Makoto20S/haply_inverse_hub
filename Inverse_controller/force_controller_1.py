import numpy as np
import time
from typing import List, Dict
from config_manager import ConfigMode, get_value

''' 
力反馈控制器
加载配置参数（墙/地板位置、刚度、阻尼等）
实现虚拟墙和虚拟地板的力反馈计算
支持动态墙（根据速度调整刚度和阻尼）
实现力滤波和限制
提供参数更新和状态查询接口
'''

class ForceFeedbackCalculator:
    def __init__(self, config_mode: ConfigMode = ConfigMode.LEADER):
        """
        力反馈计算器
        
        Args:
            config_mode: 配置模式，默认为leader（主端）
        """
        self.config_mode = config_mode
        
        # 从配置文件读取所有参数
        self._load_config_parameters()
        
        # 墙/地板基础参数
        self.wall_x = self.clamp(self.wall_x_config, -self.wall_position_limit, self.wall_position_limit)
        self.wall_y = self.clamp(self.wall_y_config, -self.wall_position_limit, self.wall_position_limit)
        self.wall_stiffness = self.clamp(self.wall_stiffness_config, self.stiffness_min, self.stiffness_max)
        self.wall_damping = self.clamp(self.wall_damping_config, self.damping_min, self.damping_max)
        
        self.floor_z = self.clamp(self.floor_z_config, -self.floor_position_limit, self.floor_position_limit)
        self.floor_stiffness = self.clamp(self.floor_stiffness_config, self.stiffness_min, self.stiffness_max)
        self.floor_damping = self.clamp(self.floor_damping_config, self.damping_min, self.damping_max)
        
        # 力反馈滤波
        self.force_filter_enabled = self.force_filter_enabled_config
        self.force_filter_cutoff_hz = self.clamp(
            self.force_filter_cutoff_hz_config, 
            self.force_filter_cutoff_min_hz, 
            self.force_filter_cutoff_max_hz
        )
        self.sampling_time = max(0.0001, self.sampling_time_config)
        self._update_filter_coefficient()
        self.filtered_force = [0.0, 0.0, 0.0]
        self.last_filter_time = time.perf_counter()
        
        # 打印参数初始化
        self.print_interval = 1.0
        self.force_limit_enabled = True
        self.force_calc_count = 0
        self.last_print_time = time.perf_counter()
        self.last_stat_time = self.last_print_time
        
        print(f"力反馈计算器初始化完成，配置模式: {config_mode.value}")
    
    def _load_config_parameters(self):
        """从配置文件加载所有参数"""
        # 力反馈启用状态
        self.force_feedback_enabled = get_value("device.force_feedback_enabled", True, self.config_mode)
        self.dynamic_wall_enabled = get_value("force_feedback.dynamic_wall_enabled", False, self.config_mode)
        
        # 墙/地板位置参数
        self.wall_x_config = get_value("force_feedback.wall_x", 2.0, self.config_mode)
        self.wall_y_config = get_value("force_feedback.wall_y", 2.0, self.config_mode)
        self.floor_z_config = get_value("force_feedback.floor_z", -2.0, self.config_mode)
        
        # 刚度参数
        self.wall_stiffness_config = get_value("force_feedback.wall_stiffness", 0, self.config_mode)
        self.floor_stiffness_config = get_value("force_feedback.floor_stiffness", 0, self.config_mode)
        
        # 阻尼参数
        self.wall_damping_config = get_value("force_feedback.wall_damping", 0, self.config_mode)
        self.floor_damping_config = get_value("force_feedback.floor_damping", 0, self.config_mode)
        
        # 滤波参数
        self.force_filter_enabled_config = get_value("force_feedback.force_filter_enabled", True, self.config_mode)
        self.force_filter_cutoff_hz_config = get_value("force_feedback.force_filter_cutoff_hz", 20.0, self.config_mode)
        self.sampling_time_config = get_value("force_feedback.sampling_time", 0.001, self.config_mode)
        
        # 控制限制参数
        self.force_limit = get_value("force_feedback.force_limit", 10.0, self.config_mode)
        self.wall_position_limit = get_value("force_feedback.wall_position_limit", 100, self.config_mode)
        self.floor_position_limit = get_value("force_feedback.floor_position_limit", -100, self.config_mode)
        self.stiffness_min = get_value("force_feedback.stiffness_min", 10.0, self.config_mode)
        self.stiffness_max = get_value("force_feedback.stiffness_max", 10000.0, self.config_mode)
        self.damping_min = get_value("force_feedback.damping_min", 0.1, self.config_mode)
        self.damping_max = get_value("force_feedback.damping_max", 100.0, self.config_mode)
        
        # 稳定性参数
        self.dead_zone_threshold = get_value("force_feedback.dead_zone_threshold", 0.001, self.config_mode)
        self.min_force_threshold = get_value("force_feedback.min_force_threshold", 0.1, self.config_mode)
        self.dead_zone_threshold_speed = get_value("force_feedback.dead_zone_threshold_speed", 0.01, self.config_mode)
        self.hysteresis_zone = get_value("force_feedback.hysteresis_zone", 0.002, self.config_mode)
        self.zero_speed_threshold = get_value("force_feedback.zero_speed_threshold", 0.001, self.config_mode)
        
        # 动态墙参数
        self.default_speed_threshold = get_value("force_feedback.default_speed_threshold", 0.1, self.config_mode)
        self.default_low_speed_stiffness_1 = get_value("force_feedback.default_low_speed_stiffness_1", 500.0, self.config_mode)
        self.default_high_speed_stiffness_1 = get_value("force_feedback.default_high_speed_stiffness_1", 2000.0, self.config_mode)
        self.default_low_speed_damping_1 = get_value("force_feedback.default_low_speed_damping_1", 5.0, self.config_mode)
        self.default_high_speed_damping_1 = get_value("force_feedback.default_high_speed_damping_1", 20.0, self.config_mode)
        self.default_low_speed_stiffness_2 = get_value("force_feedback.default_low_speed_stiffness_2", 500.0, self.config_mode)
        self.default_high_speed_stiffness_2 = get_value("force_feedback.default_high_speed_stiffness_2", 2000.0, self.config_mode)
        self.default_low_speed_damping_2 = get_value("force_feedback.default_low_speed_damping_2", 5.0, self.config_mode)
        self.default_high_speed_damping_2 = get_value("force_feedback.default_high_speed_damping_2", 20.0, self.config_mode)
        self.default_damping_factor = get_value("force_feedback.default_damping_factor", 0.5, self.config_mode)
        
        # 滤波限制参数
        self.force_filter_cutoff_min_hz = get_value("force_feedback.force_filter_cutoff_min_hz", 1.0, self.config_mode)
        self.force_filter_cutoff_max_hz = get_value("force_feedback.force_filter_cutoff_max_hz", 100.0, self.config_mode)
    
    def _update_filter_coefficient(self):
        """更新滤波系数"""
        fc = self.force_filter_cutoff_hz
        T = self.sampling_time
        self.force_filter_alpha = 1.0 - np.exp(-2.0 * np.pi * fc * T)
        self.force_filter_alpha = np.clip(self.force_filter_alpha, 0.001, 0.999)
    
    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(value, max_val))
    
    def dead_zone_velocity(self, vel: List[float]) -> List[float]:
        processed_vel = []
        for i in range(3):
            if abs(vel[i]) < self.zero_speed_threshold:
                processed_vel.append(0.0)
            elif abs(vel[i]) > self.dead_zone_threshold_speed:
                processed_vel.append((abs(vel[i]) - self.dead_zone_threshold_speed) * np.sign(vel[i]))
            else:
                processed_vel.append(0.0)
        return processed_vel
    
    def _smooth_penetration(self, penetration: float) -> float:
        if penetration < self.dead_zone_threshold:
            return 0.0
        elif self.dead_zone_threshold <= penetration < self.dead_zone_threshold + self.hysteresis_zone:
            # 迟滞区内线性插值
            ratio = (penetration - self.dead_zone_threshold) / self.hysteresis_zone
            return penetration * ratio
        else:
            return penetration - self.hysteresis_zone
    
    def _filter_small_force(self, force: List[float]) -> List[float]:
        return [f if abs(f) >= self.min_force_threshold else 0.0 for f in force]
    
    def calculate_wall_force(self, pos: List[float], vel: List[float]) -> List[float]:
        return [0.0, 0.0, 0.0]
        # force = [0.0, 0.0, 0.0]

        # # X方向墙力计算
        # if pos[0] > self.wall_x:
        #     penetration = pos[0] - self.wall_x
        #     penetration_smooth = self._smooth_penetration(penetration)  # 平滑穿透量
        #     if penetration_smooth > 0:
        #         if self.dynamic_wall_enabled:
        #             speed_x = abs(vel[0])
        #             speed_threshold = self.default_speed_threshold
        #             # 插值因子边界保护
        #             t = np.clip((speed_x / speed_threshold) ** 2, 0.0, 1.0)
        #             stiffness_x = self.default_low_speed_stiffness_1 * (1 - t) + self.default_high_speed_stiffness_1 * t
        #             damping_x = self.default_low_speed_damping_1 * (1 - t) + self.default_high_speed_damping_1 * t  # 平滑插值
        #         else:
        #             stiffness_x = self.wall_stiffness
        #             damping_x = self.wall_damping
        #         damping_factor_x = 1.0 if vel[0] > 0 else self.default_damping_factor
        #         force[0] = -stiffness_x * penetration_smooth - damping_x * damping_factor_x * vel[0]

        # # Y方向墙力计算（同上）
        # if pos[1] > self.wall_y:
        #     penetration = pos[1] - self.wall_y
        #     penetration_smooth = self._smooth_penetration(penetration)
        #     if penetration_smooth > 0:
        #         if self.dynamic_wall_enabled:
        #             speed_y = abs(vel[1])
        #             speed_threshold = self.default_speed_threshold
        #             t = np.clip((speed_y / speed_threshold) ** 2, 0.0, 1.0)
        #             stiffness_y = self.default_low_speed_stiffness_1 * (1 - t) + self.default_high_speed_stiffness_1 * t
        #             damping_y = self.default_low_speed_damping_1 * (1 - t) + self.default_high_speed_damping_1 * t
        #         else:
        #             stiffness_y = self.wall_stiffness
        #             damping_y = self.wall_damping
        #         damping_factor_y = 1.0 if vel[1] > 0 else self.default_damping_factor
        #         force[1] = -stiffness_y * penetration_smooth - damping_y * damping_factor_y * vel[1]
            
        # return force
    
    def calculate_floor_force(self, pos: List[float], vel: List[float]) -> List[float]:
        # ★★★ 力矩墙已禁用 - 直接返回零力 ★★★
        return [0.0, 0.0, 0.0]
        # force = [0.0, 0.0, 0.0]
        
        # # Z方向地板力计算
        # if pos[2] < self.floor_z:
        #     penetration = self.floor_z - pos[2]
        #     penetration_smooth = self._smooth_penetration(penetration)
        #     if penetration_smooth > 0:
        #         if self.dynamic_wall_enabled:
        #             speed_z = abs(vel[2])
        #             speed_threshold = self.default_speed_threshold
        #             t = np.clip((speed_z / speed_threshold) ** 2, 0.0, 1.0)
        #             stiffness_z = self.default_low_speed_stiffness_2 * (1 - t) + self.default_high_speed_stiffness_2 * t
        #             damping_z = self.default_low_speed_damping_2 * (1 - t) + self.default_high_speed_damping_2 * t
        #         else:
        #             stiffness_z = self.floor_stiffness
        #             damping_z = self.floor_damping
        #         damping_factor_z = 1.0 if vel[2] < 0 else self.default_damping_factor
        #         force[2] = stiffness_z * penetration_smooth - damping_z * damping_factor_z * vel[2]
            
        # return force
    
    def _apply_low_pass_filter(self, current_force: List[float]) -> List[float]:

        # 计算当前输入的力的模长
        force_mag = sum(f**2 for f in current_force) ** 0.5
        # ★ 如果输入的力已经为0（或极其微弱），直接切断，消除滤波拖尾
        if force_mag < 0.01:
            self.filtered_force = [0.0, 0.0, 0.0]
            self.last_filter_time = time.perf_counter()
            return [0.0, 0.0, 0.0]
        
        filtered = [0.0, 0.0, 0.0]
        for i in range(3):
            filtered[i] = self.force_filter_alpha * current_force[i] + (1 - self.force_filter_alpha) * self.filtered_force[i]
        self.filtered_force = filtered
        self.last_filter_time = time.perf_counter()
        return filtered
    
    def calculate_total_force(self, 
                             pos: List[float], 
                             vel: List[float],
                             external_force: List[float] = None) -> List[float]:
        """
        执行顺序（外部算力融合→算力→滤波→小力过滤→力限制）
        """
        vel_processed = self.dead_zone_velocity(vel)
        
        # 计算墙力和地板力
        # wall_force = self.calculate_wall_force(pos, vel_processed)
        # floor_force = self.calculate_floor_force(pos, vel_processed)
        # total_force = [wall_force[0], wall_force[1], floor_force[2]]
        total_force = [0.0, 0.0, 0.0]
        
        # 叠加外部反馈力（例如来自 MuJoCo 的擦桌子摩擦力、法向支持力）
        if external_force is not None and len(external_force) == 3:
            total_force[0] += external_force[0]
            total_force[1] += external_force[1]
            total_force[2] += external_force[2]

        # ★★★ 新增：快速归零机制 ★★★
        # 当总力很小（接近零）时，强制加速滤波器衰减，
        # 防止低通滤波器拖尾导致力反馈"粘连"
        if self.force_filter_enabled:
            total_mag = (total_force[0]**2 + total_force[1]**2 + total_force[2]**2) ** 0.5
            if total_mag < self.min_force_threshold:
                # 输入已经很小 → 强制清除滤波器状态，不让历史值拖尾
                self.filtered_force = [0.0, 0.0, 0.0]
                total_force = [0.0, 0.0, 0.0]
            else:
                total_force = self._apply_low_pass_filter(total_force)
        
        # if self.force_filter_enabled:
        #     total_force = self._apply_low_pass_filter(total_force)
        
        total_force = self._filter_small_force(total_force)
        
        if self.force_limit_enabled:
            total_force = self.limit_force(total_force)
        
        return total_force
    
    def limit_force(self, force: List[float]) -> List[float]:
        force_magnitude = np.sqrt(np.sum(np.square(force)))
        if force_magnitude > self.force_limit and force_magnitude > 1e-6:
            scale = self.force_limit / force_magnitude
            force = [f * scale for f in force]
        return [self.clamp(f, -self.force_limit, self.force_limit) for f in force]
    
    def limit_force_simple(self, force: List[float]) -> List[float]:
        limited_force = [
            max(-self.force_limit, min(self.force_limit, force[0])),
            max(-self.force_limit, min(self.force_limit, force[1])),
            max(-self.force_limit, min(self.force_limit, force[2]))
        ]
        return limited_force
    
    def update_parameters(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                if key in ['wall_x', 'wall_y']:
                    value = self.clamp(value, -self.wall_position_limit, self.wall_position_limit)
                elif key == 'floor_z':
                    value = self.clamp(value, -self.floor_position_limit, self.floor_position_limit)
                elif key in ['wall_stiffness', 'floor_stiffness']:
                    value = self.clamp(value, self.stiffness_min, self.stiffness_max)
                elif key in ['wall_damping', 'floor_damping']:
                    value = self.clamp(value, self.damping_min, self.damping_max)
                elif key == 'force_filter_cutoff_hz':
                    value = self.clamp(value, self.force_filter_cutoff_min_hz, self.force_filter_cutoff_max_hz)
                    self.force_filter_cutoff_hz = value
                    self._update_filter_coefficient()
                    self.filtered_force = [0.0, 0.0, 0.0]
                elif key == 'sampling_time':
                    value = max(0.0001, value)
                    self.sampling_time = value
                    self._update_filter_coefficient()
                elif key == 'force_filter_enabled':
                    self.force_filter_enabled = bool(value)
                else:
                    setattr(self, key, value)
    
    def get_parameters(self) -> Dict:
        return {
            'wall_x': self.wall_x,
            'wall_y': self.wall_y,
            'floor_z': self.floor_z,
            'wall_stiffness': self.wall_stiffness,
            'floor_stiffness': self.floor_stiffness,
            'wall_damping': self.wall_damping,
            'floor_damping': self.floor_damping,
            'force_filter_cutoff_hz': self.force_filter_cutoff_hz,
            'sampling_time': self.sampling_time,
            'force_filter_alpha': self.force_filter_alpha,
            'force_filter_enabled': self.force_filter_enabled,
            'force_limit': self.force_limit,
            'dead_zone_threshold': self.dead_zone_threshold,
            'zero_speed_threshold': self.zero_speed_threshold,
            'min_force_threshold': self.min_force_threshold,
            'dynamic_wall_enabled': self.dynamic_wall_enabled
        }
    
    def print_status(self, current_pos: List[float] = None, current_force: List[float] = None):
        current_time = time.perf_counter()
        if (current_time - self.last_print_time) < self.print_interval:
            return
        
        self.last_print_time = current_time
        if current_pos is not None and current_force is not None:
            status_str = f"位置:[{current_pos[0]*1e3:+.1f},{current_pos[1]*1e3:+.1f},{current_pos[2]*1e3:+.1f}]mm | 力:[{current_force[0]:+.1f},{current_force[1]:+.1f},{current_force[2]:+.1f}]N"
            print(f"\r{status_str}", end="", flush=True)
    
    def print_parameters(self):
        print("\n" + "="*80)
        print(f"力反馈参数配置（{self.config_mode.value.upper()}模式）")
        print("="*80)
        print(f"墙参数:")
        print(f"  X墙位置: {self.wall_x:+.3f} m | Y墙位置: {self.wall_y:+.3f} m")
        print(f"  墙刚度: {self.wall_stiffness:.0f} N/m | 墙阻尼: {self.wall_damping:.1f} N·s/m")
        print(f"地板参数:")
        print(f"  Z位置: {self.floor_z:+.3f} m | 地板刚度: {self.floor_stiffness:.0f} N/m | 地板阻尼: {self.floor_damping:.1f} N·s/m")
        print(f"稳定性参数:")
        print(f"  位置死区: {self.dead_zone_threshold*1e3:.1f}mm | 零速区: {self.zero_speed_threshold*1e3:.1f}mm/s | 最小力阈值: {self.min_force_threshold:.1f}N")
        print(f"滤波参数:")
        print(f"  截止频率: {self.force_filter_cutoff_hz:.0f}Hz | 滤波系数: {self.force_filter_alpha:.3f} | 启用: {self.force_filter_enabled}")
        print(f"动态墙: {'启用' if self.dynamic_wall_enabled else '禁用'}")
        print(f"力限制: {self.force_limit:.1f}N")
        print("="*80)


# 向后兼容的全局变量
IS_FORCE_FEEDBACK_ENABLED = True  # 保持向后兼容，实际使用force_feedback_enabled配置
IS_DYNAMIC_WALL_ENABLED = False   # 保持向后兼容，实际使用dynamic_wall_enabled配置


if __name__ == "__main__":
    # 测试leader配置
    print("测试leader配置:")
    calculator_leader = ForceFeedbackCalculator(ConfigMode.LEADER)
    calculator_leader.print_parameters()
    
    # 测试follower配置
    print("\n\n测试follower配置:")
    calculator_follower = ForceFeedbackCalculator(ConfigMode.FOLLOWER)
    calculator_follower.print_parameters()
    
    # 模拟低速场景（速度0.01m/s，带位置噪声）
    test_pos = [0.101, 0.101, -0.051]  # 轻微穿透
    test_vel = [0.01, 0.01, -0.01]     # 低速运动
    
    # 连续计算100次，验证力输出稳定性
    print("\n\n低速场景力输出测试（100次计算，含位置噪声）：")
    force_history = []
    for i in range(100):
        # 加入微小位置噪声（模拟硬件实际采集）
        noisy_pos = [
            test_pos[0] + np.random.normal(0, 0.0003),
            test_pos[1] + np.random.normal(0, 0.0003),
            test_pos[2] + np.random.normal(0, 0.0003)
        ]
        force = calculator_leader.calculate_total_force(noisy_pos, test_vel)
        force_history.append(force)
        if i % 10 == 0:
            calculator_leader.print_status(current_pos=noisy_pos, current_force=force)
    
    # 统计稳定性（标准差越小，抖动越少）
    force_np = np.array(force_history)
    print(f"\n\n=== 低速力输出统计 ===")
    print(f"X轴力：均值={np.mean(force_np[:,0]):.2f}N | 标准差={np.std(force_np[:,0]):.3f}N")
    print(f"Y轴力：均值={np.mean(force_np[:,1]):.2f}N | 标准差={np.std(force_np[:,1]):.3f}N")
    print(f"Z轴力：均值={np.mean(force_np[:,2]):.2f}N | 标准差={np.std(force_np[:,2]):.3f}N")
    print("注：标准差<0.05N表示抖动已显著改善")