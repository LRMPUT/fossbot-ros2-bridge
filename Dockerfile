FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf-transformations \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rqt-common-plugins \
      ros-jazzy-rviz2 \
      ros-jazzy-slam-toolbox \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      iproute2 iputils-ping netcat-openbsd \
      libnss-mdns \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i 's/^hosts:.*/hosts:          files mdns4_minimal [NOTFOUND=return] dns/' /etc/nsswitch.conf

WORKDIR /ws
COPY ws/src /ws/src

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
