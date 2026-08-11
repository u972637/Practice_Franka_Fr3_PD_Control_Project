# franka_ws

FR3(Franka Research 3) 로봇팔을 MuJoCo 시뮬레이션에서 텔레오퍼레이션하기 위한 ROS 2 Humble 워크스페이스입니다. `franka_ik_bridge` 패키지가 MuJoCo(키보드/마우스로 타깃 포즈 조작)와 MoveIt 2(IK 풀이)를 연결합니다.

## 사전 요구사항

- Ubuntu 22.04
- ROS 2 Humble ([설치 가이드](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
- MoveIt 2 (아래 rosdep 단계에서 함께 설치됨)

MuJoCo 3.4.0은 `mujoco-3.4.0/`에 워크스페이스 안에 이미 포함되어 있어 별도 설치가 필요 없습니다.

## 워크스페이스 준비

`src/franka_ros2`, `src/franka_description`는 Franka 공식 저장소로, 이 저장소에는 포함되어 있지 않습니다(`.gitignore` 처리). 클론 후 직접 받아야 합니다:

```bash
cd ~/franka_ws/src
git clone -b humble https://github.com/frankarobotics/franka_ros2.git
git clone https://github.com/frankaemika/franka_description.git
```

참고: 현재 개발은 아래 커밋 기준으로 진행되었습니다.
- `franka_ros2` (`humble` 브랜치): `7ef0ab0`
- `franka_description` (`main` 브랜치): `2b4c24c`

## 의존성 설치

```bash
cd ~/franka_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`franka_ik_bridge/package.xml`에 `moveit_core`, `moveit_ros_planning`, `moveit_ros_planning_interface`가 선언되어 있어 rosdep이 필요한 MoveIt 2 apt 패키지를 자동으로 설치합니다.

## 빌드 및 실행

```bash
source /opt/ros/humble/setup.bash
cd ~/franka_ws
colcon build --symlink-install --packages-select franka_ik_bridge
```

또는 빌드부터 실행까지 한 번에:

```bash
./run_standalone.sh
```

MuJoCo 창이 뜨면 WASDQE(이동), UIOJKL(회전)로 타깃 포즈를 조작하고, IK로 풀린 관절각이 시뮬레이션에 반영됩니다.
