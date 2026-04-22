# 使用 ROS 2 Jazzy 基础镜像（基于 Ubuntu 24.04）
FROM ros:jazzy-ros-base

# 设置非交互式安装，避免时区等提示卡住
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 安装系统依赖：编译工具 + CycloneDDS
RUN apt-get update && apt-get install -y \
    bash-completion \
    curl \
    gdb \
    git \
    nano \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    sudo \
    vim \
    python3-pip \
    build-essential \
    python3-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /HAPLY_INVERSE

# 先复制依赖文件，利用 Docker 层缓存
COPY requirements.txt .
RUN pip install --no-cache-dir --break-system-packages -r requirements.txt

# 复制项目代码
COPY device_node.py .
COPY Inverse_controller/ ./Inverse_controller/
COPY ros_ws/src/ ./ros_ws/src/

# 切换默认 shell 为 bash，否则 setup.bash 会报 Bad substitution
SHELL ["/bin/bash", "-c"]
# 编译自定义消息包
RUN source /opt/ros/jazzy/setup.bash && \
    cd /HAPLY_INVERSE/ros_ws && \
    colcon build --packages-select teleop_msgs

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /HAPLY_INVERSE/ros_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
