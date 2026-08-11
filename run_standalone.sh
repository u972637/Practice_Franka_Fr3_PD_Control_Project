#!/bin/bash

# 1. 환경 설정
ROS_SETUP=/opt/ros/humble/setup.bash
WS_PATH=~/franka_ws
PKG_NAME=franka_ik_bridge

# 빌드 전에 ROS 환경을 반드시 먼저 불러와야 합니다
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
    echo "ROS 2 Humble 환경을 불러왔습니다."
else
    echo "에러: ROS 2 설치 경로를 찾을 수 없습니다 ($ROS_SETUP)"
    exit 1
fi

echo "빌드를 시작합니다..."
cd $WS_PATH
colcon build --symlink-install --packages-select $PKG_NAME

# 빌드 결과 확인
if [ $? -ne 0 ]; then
    echo "❌ 빌드 실패! 위 에러 메시지를 확인하세요."
    read -p "엔터를 누르면 종료됩니다..."
    exit 1
fi

source install/setup.bash
# ... 이후 실행 로직

# 3. 이전 프로세스 종료
pkill -f mujoco_ros_bridge
pkill -f franka_ik_node

# 4. 실행
echo "시스템 시작 중..."
gnome-terminal --tab --title="T3_Bridge" -- bash -c "source /opt/ros/humble/setup.bash; source $WS_PATH/install/setup.bash; ros2 run $PKG_NAME mujoco_ros_bridge; exec bash"
sleep 1
gnome-terminal --tab --title="T2_IK" -- bash -c "source /opt/ros/humble/setup.bash; source $WS_PATH/install/setup.bash; ros2 run $PKG_NAME franka_ik_node; exec bash"

echo "완료. MuJoCo 창에서 로봇을 조종하세요."
