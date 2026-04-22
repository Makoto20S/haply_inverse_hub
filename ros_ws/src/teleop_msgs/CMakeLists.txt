cmake_minimum_required(VERSION 3.8)
project(teleop_msgs)                    # 项目名，必须和 package.xml 里的 <name> 一致

# ===== 找依赖 =====
find_package(ament_cmake REQUIRED)               # ROS 2 构建系统
find_package(std_msgs REQUIRED)                  # 因为 .msg 里引用了 std_msgs/Header
find_package(rosidl_default_generators REQUIRED) # 消息代码生成器

# ===== 核心：声明要生成哪些消息 =====
rosidl_generate_interfaces(${PROJECT_NAME}       # ${PROJECT_NAME} = "teleop_msgs"
  "msg/DeviceState.msg"                          # 列出所有 .msg 文件（相对路径）
  DEPENDENCIES std_msgs                          # .msg 里引用的外部消息包
)

ament_package()                                  # 固定结尾，注册为 ament 包

