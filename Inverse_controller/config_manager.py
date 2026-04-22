import os
import tomlkit as toml
from typing import Dict, Any, Optional
from enum import Enum

# 配置管理器
class ConfigMode(Enum):
    """配置模式枚举"""
    LEADER = "leader"      # 主端配置
    FOLLOWER = "follower"  # 从端配置


class ConfigManager:
    """配置管理器"""
    
    def __init__(self, config_dir: str | None = None):
        """
        初始化配置管理器
        
        Args:
            config_dir: 配置文件目录
        """
        # ★★★ 关键修复：用绝对路径，不依赖工作目录 ★★★
        if config_dir is None:
            config_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "config"
            )
        
        self.config_dir = config_dir
        self.configs: Dict[str, Dict[str, Any]] = {}
        self.current_mode: Optional[ConfigMode] = None
        
        # 确保配置目录存在
        os.makedirs(config_dir, exist_ok=True)
        
        # ★★★ 调试输出：启动时打印实际路径，确认读对了 ★★★
        print(f"[ConfigManager] 配置目录: {os.path.abspath(config_dir)}")
        
        # 初始化默认配置
        self._init_default_configs()
        
    def _init_default_configs(self):
        """初始化默认配置"""
        # leader（主端）默认配置
        leader_default = {
            # 网络配置
            "network": {
                "uri": "ws://localhost:10001",
                "max_retries": 3,
                "timeout": 5.0,
                "reconnect_interval": 1.0
            },
            # 坐标系配置
            "coordinate": {
                "origin": "workspace_center",
                "basis": "XYZ",
                "scale_factor": 1.0
            },
            # 设备配置
            "device": {
                "data_refresh_rate": 0.0, # 刷新率
                "force_feedback_enabled": True, # 力反馈
                "inverse3_device_id": "", # Inverse3设备ID，null表示自动检测
                "verse_grip_device_id": "" # VerseGrip设备ID，null表示自动检测
            },
            # 力反馈配置
            "force_feedback": {
                "wall_x": 100.0,
                "wall_y": 100.0,
                "floor_z": -100.0,
                "wall_stiffness": 0,
                "floor_stiffness": 0,
                "wall_damping": 0,
                "floor_damping": 0,
                "force_filter_cutoff_hz": 20.0,
                "sampling_time": 0.001,
                "force_filter_enabled": True,
                "force_limit": 10.0,
                "dead_zone_threshold": 0.001,
                "zero_speed_threshold": 0.001,
                "min_force_threshold": 0.1,
                "dynamic_wall_enabled": False,
                "default_speed_threshold": 0.1,
                "default_low_speed_stiffness_1": 500.0,
                "default_high_speed_stiffness_1": 2000.0,
                "default_low_speed_damping_1": 5.0,
                "default_high_speed_damping_1": 20.0,
                "default_low_speed_stiffness_2": 500.0,
                "default_high_speed_stiffness_2": 2000.0,
                "default_low_speed_damping_2": 5.0,
                "default_high_speed_damping_2": 20.0,
                "default_damping_factor": 0.5,
                "hysteresis_zone": 0.002,
                "dead_zone_threshold_speed": 0.01,
                "wall_position_limit": 0.5,
                "floor_position_limit": 0.5,
                "stiffness_min": 10.0,
                "stiffness_max": 10000.0,
                "damping_min": 0.1,
                "damping_max": 100.0,
                "force_filter_cutoff_min_hz": 1.0,
                "force_filter_cutoff_max_hz": 100.0
            },
            # 机器人配置
            "robot": {
                "robot_type": "inverse3",
                "tool_offset": [0, 0, 0.21],
                "home_position": [0.0, 0.785, 0.0, -0.785, 0.0, 1.571, 0.0]
            },
            # 可视化配置
            "visualization": {
                "update_interval": 0.016,
                "window_width": 800,
                "window_height": 600,
                "background_color": [0.1, 0.1, 0.1, 1.0]
            }
        }
        
        # follower（从端）默认配置
        follower_default = {
            "network": {
                "uri": "ws://localhost:10002", # 从端10002
                "max_retries": 3,
                "timeout": 5.0,
                "reconnect_interval": 1.0
            },
            "coordinate": {
                "origin": "workspace_center",
                "basis": "XYZ",
                "scale_factor": 1.0
            },
            "device": {
                "data_refresh_rate": 0.0,
                "force_feedback_enabled": False,
                "inverse3_device_id": "",
                "verse_grip_device_id": ""
            },
            # 力反馈配置
            "force_feedback": {
                "wall_x": 0.1,
                "wall_y": 0.1,
                "floor_z": -0.1,
                "wall_stiffness": 500.0,
                "floor_stiffness": 500.0,
                "wall_damping": 5.0,
                "floor_damping": 5.0,
                "force_filter_cutoff_hz": 10.0,
                "sampling_time": 0.001,
                "force_filter_enabled": False,   
                "force_limit": 5.0,
                "dead_zone_threshold": 0.002,
                "zero_speed_threshold": 0.002,
                "min_force_threshold": 0.2,
                "dynamic_wall_enabled": False
            },
            "robot": {
                "robot_type": "franka_fr3_v2", # 机器人类型：Franka FR3 v2
                "tool_offset": [0, 0, 0.1], # 工具偏移（米）
                "home_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 机器人初始关节角度
            },
            # 可视化配置
            "visualization": {
                "update_interval": 0.033, # 可视化更新间隔（秒）,约30fps
                "window_width": 640, # 窗口宽度
                "window_height": 480, # 窗口高度
                "background_color": [0.2, 0.2, 0.2, 1.0] # 背景颜色
            }
        }
        
        self.default_configs = {
            ConfigMode.LEADER: leader_default,
            ConfigMode.FOLLOWER: follower_default
        }
    
    def load_config(self, mode: ConfigMode, create_if_missing: bool = True) -> Dict[str, Any]:
        """
        加载指定模式的配置
        
        Args:
            mode: 配置模式
            create_if_missing: 如果配置文件不存在，是否创建默认配置
            
        Returns:
            配置字典
        """
        config_file = os.path.join(self.config_dir, f"{mode.value}.toml")

        # ★★★ 打印实际加载的文件路径 ★★★
        print(f"[ConfigManager] 尝试加载: {os.path.abspath(config_file)}")
        
        if os.path.exists(config_file):
            # 加载现有配置
            with open(config_file, 'r', encoding='utf-8') as f:
                config = toml.load(f)
        elif create_if_missing:
            # 创建默认配置
            config = self.default_configs[mode].copy()
            self.save_config(mode, config)
        else:
            raise FileNotFoundError(f"配置文件不存在: {config_file}")
        
        self.configs[mode.value] = config
        self.current_mode = mode
        return config
    
    def save_config(self, mode: ConfigMode, config: Dict[str, Any]):
        """
        保存配置到文件
        
        Args:
            mode: 配置模式
            config: 配置字典
        """
        config_file = os.path.join(self.config_dir, f"{mode.value}.toml")
        
        with open(config_file, 'w', encoding='utf-8') as f:
            toml.dump(config, f)
        
        self.configs[mode.value] = config
    
    def get_config(self, mode: Optional[ConfigMode] = None) -> Dict[str, Any]:
        """
        获取当前或指定模式的配置
        
        Args:
            mode: 配置模式，如果为None则使用当前模式
            
        Returns:
            配置字典
        """
        if mode is None:
            if self.current_mode is None:
                raise ValueError("未设置当前配置模式")
            mode = self.current_mode
        
        if mode.value not in self.configs:
            self.load_config(mode)
        
        return self.configs[mode.value]
    
    def get_value(self, key_path: str, default: Any = None, mode: Optional[ConfigMode] = None) -> Any:
        """
        获取配置值
        
        Args:
            key_path: 键路径，使用点号分隔，如 "network.uri"
            default: 默认值
            mode: 配置模式
            
        Returns:
            配置值
        """
        config = self.get_config(mode)
        
        # 按点号分割键路径
        keys = key_path.split('.')
        value = config
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def set_value(self, key_path: str, value: Any, mode: Optional[ConfigMode] = None):
        """
        设置配置值
        
        Args:
            key_path: 键路径，使用点号分隔
            value: 值
            mode: 配置模式
        """
        config = self.get_config(mode)
        
        # 按点号分割键路径
        keys = key_path.split('.')
        current = config
        
        # 遍历到倒数第二个键
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        # 设置最后一个键的值
        current[keys[-1]] = value
        
        # 保存配置
        self.save_config(mode or self.current_mode, config)
    
    def switch_mode(self, mode: ConfigMode):
        """
        切换当前配置模式
        
        Args:
            mode: 新的配置模式
        """
        self.current_mode = mode
        if mode.value not in self.configs:
            self.load_config(mode)
    
    def create_default_configs(self):
        """创建所有默认配置文件"""
        for mode in ConfigMode:
            config_file = os.path.join(self.config_dir, f"{mode.value}.toml")
            if not os.path.exists(config_file):
                config = self.default_configs[mode].copy()
                self.save_config(mode, config)
                print(f"已创建默认配置文件: {config_file}")
    
    def print_config(self, mode: Optional[ConfigMode] = None):
        """
        打印配置信息
        
        Args:
            mode: 配置模式
        """
        config = self.get_config(mode)
        mode_str = (mode or self.current_mode).value if mode or self.current_mode else "unknown"
        
        print(f"\n{'='*60}")
        print(f"配置模式: {mode_str.upper()}")
        print(f"{'='*60}")
        
        for section, values in config.items():
            print(f"\n[{section}]")
            for key, value in values.items():
                if isinstance(value, dict):
                    print(f"  {key}:")
                    for sub_key, sub_value in value.items():
                        print(f"    {sub_key}: {sub_value}")
                else:
                    print(f"  {key}: {value}")
        
        print(f"\n{'='*60}")


# 全局配置管理器实例
_config_manager: Optional[ConfigManager] = None


def get_config_manager() -> ConfigManager:
    """获取全局配置管理器实例"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager()
    return _config_manager


def load_config(mode: ConfigMode) -> Dict[str, Any]:
    """加载配置的快捷函数"""
    return get_config_manager().load_config(mode)


def get_config(mode: Optional[ConfigMode] = None) -> Dict[str, Any]:
    """获取配置的快捷函数"""
    return get_config_manager().get_config(mode)


def get_value(key_path: str, default: Any = None, mode: Optional[ConfigMode] = None) -> Any:
    """获取配置值的快捷函数"""
    return get_config_manager().get_value(key_path, default, mode)


def set_value(key_path: str, value: Any, mode: Optional[ConfigMode] = None):
    """设置配置值的快捷函数"""
    get_config_manager().set_value(key_path, value, mode)


def switch_mode(mode: ConfigMode):
    """切换配置模式的快捷函数"""
    get_config_manager().switch_mode(mode)

