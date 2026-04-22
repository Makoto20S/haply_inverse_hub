# 文件: ros_ws/src/teleop_nodes/setup.py
"""
ROS2 Python 包的安装配置文件

关键作用：
1. 定义包名、版本、依赖
2. 声明可执行节点的入口点（entry_points）—— 这是 ros2 run 能找到节点的关键
3. 声明要安装的数据文件（launch、config、resource）
"""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_nodes'

setup(
    name=package_name,
    version='0.1.0',
    # ---- 自动发现 Python 模块 ----
    # find_packages() 会找到 teleop_nodes/ 目录（因为里面有 __init__.py）
    packages=find_packages(exclude=['test']),

    # ---- 数据文件：安装到 share/ 目录下 ----
    # 这些文件不是 Python 代码，但 ROS2 运行时需要能找到它们
    # 格式：(安装目标目录, 源文件列表)
    data_files=[
        # 1) ament 索引标记（必须有，少了 ros2 pkg list 看不到这个包）
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)],
        ),
        # 2) package.xml（必须有，ros2 pkg xml 要读它）
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        # 3) launch 文件 —— ros2 launch teleop_nodes teleop_launch.py 能找到
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
            # 这个 glob 模式匹配: *.launch.py, *.launch.xml, *.launch.yaml
            # 以及简写的 *launch.py 等
        ),
        # 4) 参数配置文件 —— launch 里 load 参数用
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],

    # ---- Python 依赖（pip 层面的，不是 ROS2 层面的）----
    install_requires=['setuptools'],

    zip_safe=True,

    # ---- 可执行节点入口点 ★★★ 最重要 ★★★ ----
    # 格式：'命令名 = 包名.模块名:函数名'
    # ros2 run teleop_nodes device_node
    #   → 实际调用 teleop_nodes.device_node 模块里的 main() 函数
    entry_points={
        'console_scripts': [
            'device_node = teleop_nodes.device_node:main',
            # 如果以后加新节点，在这里追加一行即可，例如：
            # 'monitor_node = teleop_nodes.monitor_node:main',
        ],
    },
)