FROM ros:humble-ros-base

# ROS 2 Navigasyon ve SLAM paketlerini kuruyoruz
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Simülasyon matematiği için
RUN pip3 install numpy

ENV ROS_DOMAIN_ID=1

# ROS ortamını otomatik yükle
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws

# Script dosyasını içeri kopyala
COPY simple_sim.py /root/ros2_ws/simple_sim.py