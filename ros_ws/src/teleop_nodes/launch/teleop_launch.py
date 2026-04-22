# 文件: ros_ws/src/teleop_nodes/launch/teleop_launch.py
"""
一键启动遥操作系统

用法:
  # 默认参数启动
  ros2 launch teleop_nodes teleop_launch.py

  # 指定自定义参数文件
  ros2 launch teleop_nodes teleop_launch.py params_file:=/path/to/my_params.yaml

  # 命令行覆盖单个参数（优先级最高）
  ros2 launch teleop_nodes teleop_launch.py force_z_desired:=-5.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    ROS2 launch 系统要求此文件必须包含这个函数，
    返回一个 LaunchDescription 对象。
    """

    # ---- 获取包的 share 目录（colcon build 后的安装路径） ----
    pkg_share = get_package_share_directory('teleop_nodes')

    # ---- 默认参数文件路径 ----
    default_params_file = os.path.join(pkg_share, 'config', 'device_params.yaml')

    # ============================================================
    # 声明 Launch 参数（可在命令行覆盖）
    # ============================================================

    # 参数文件路径
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='YAML 参数文件的完整路径'
    )

    # 常用的快捷覆盖参数（不想改 YAML 时用命令行直接改）
    force_z_arg = DeclareLaunchArgument(
        'force_z_desired',
        default_value='-3.0',
        description='Z 轴期望力 (N)，负值=向下推'
    )

    position_scale_arg = DeclareLaunchArgument(
        'position_scale',
        default_value='1.0',
        description='位置缩放因子'
    )

    # ============================================================
    # 节点定义
    # ============================================================

    device_node = Node(
        package='teleop_nodes',
        executable='device_node',        # 对应 setup.py 里 entry_points 的 key
        name='device_node',              # 节点名，YAML 里的顶层 key 必须与此一致
        output='screen',                 # 日志输出到终端（方便调试）
        emulate_tty=True,                # 保留日志的颜色和格式
        parameters=[
            # 1) 先加载 YAML 文件（基础参数）
            LaunchConfiguration('params_file'),
            # 2) 再用命令行参数覆盖（优先级更高）
            #    只列出你希望能从命令行快速改的参数
            {
                'force_z_desired': LaunchConfiguration('force_z_desired'),
                'position_scale': LaunchConfiguration('position_scale'),
            },
        ],
        # ---- 话题重映射（如果需要改话题名） ----
        # remappings=[
        #     ('/device/pose', '/my_robot/target_pose'),
        # ],
    )

    # ============================================================
    # 组装并返回
    # ============================================================

    return LaunchDescription([
        # Launch 参数声明（必须放在使用它们的 Node 之前）
        params_file_arg,
        force_z_arg,
        position_scale_arg,

        # 启动提示
        LogInfo(msg=['=========================================']),
        LogInfo(msg=['  Haply Inverse3 遥操作节点启动中...']),
        LogInfo(msg=['  参数文件: ', LaunchConfiguration('params_file')]),
        LogInfo(msg=['=========================================']),

        # 节点
        device_node,
    ])