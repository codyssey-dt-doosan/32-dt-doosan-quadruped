# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
# osrf/ros:jazzy-desktop은 amd64만 배포돼 Apple Silicon에서 에뮬레이션으로 돈다 — 멀티 아키텍처 ros-base + ros-jazzy-desktop(같은 내용)
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Seoul \
    GZ_VERSION=harmonic \
    ROS_DISTRO=jazzy

# Gazebo Harmonic 은 ros-jazzy-ros-gz 가 vendor 패키지(ros-jazzy-gz-*-vendor)로 함께 설치한다.
# gz-harmonic 은 OSRF apt 저장소에만 있어 여기서는 설치하지 않는다.
# 미러가 간헐적으로 400/해시 오류를 내므로 다운로드를 재시도한다.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-yaml \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-aiohttp \
    python3-scipy \
    python3-jinja2 \
    python3-joblib \
    && rm -rf /var/lib/apt/lists/*

# walk_node 힘 QP(osqp 1.x API). --no-deps: pip가 setuptools를 올리면 colcon --symlink-install이 깨진다
RUN pip install --no-cache-dir --no-deps --break-system-packages osqp==1.1.3

WORKDIR /workspace
COPY . /workspace
# Windows(Git 기본 autocrlf)로 clone하면 .sh가 CRLF라 bash가 문법 오류로 죽는다
RUN sed -i 's/\r$//' /workspace/unity/tools/*.sh

RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

SHELL ["/bin/bash", "-c"]
# .bashrc 는 비대화형(bash -lc, docker run ... 명령)에서 맨 앞에서 return 하므로
# 워크스페이스는 엔트리포인트에서 로드한다. 대화형 셸용으로 .bashrc 에도 남긴다.
RUN sed -i 's|^exec "\$@"|source /workspace/install/setup.bash --\nexec "$@"|' /ros_entrypoint.sh \
    && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["bash", "-lc", "ros2 launch simulation full_system.launch.py"]
