# franka_ws

FR3(Franka Research 3) 로봇팔을 MuJoCo 시뮬레이션에서 텔레오퍼레이션하기 위한 ROS 2 Humble 워크스페이스입니다. 두 가지 독립적인 제어 방식이 들어있습니다.

- **`src/franka_ik_bridge/`** — MuJoCo(키보드/마우스로 타깃 포즈 조작)와 MoveIt 2(IK 풀이)를 연결하는 ROS 2 패키지. 목표 포즈를 IK로 풀어 관절각을 직접 지정.
- **`previous_pd_project/`** — IK 솔버 없이 자코비안 전치(Jacobian Transpose) + 작업공간 스프링-댐퍼(PD) 힘 제어로 엔드이펙터가 타깃을 따라가게 한 이전 실험. ROS/MoveIt에 의존하지 않는 순수 MuJoCo+GLFW C++ 프로그램.

## 워크스페이스 구조

```
franka_ws/
├── mujoco-3.4.0/           # MuJoCo 3.4.0 배포판 (벤더링됨, Apache-2.0 LICENSE 포함)
├── previous_pd_project/    # 자코비안 전치 PD 제어 실험 (ROS 불필요)
├── src/
│   ├── franka_ik_bridge/   # MuJoCo <-> MoveIt2 IK 브릿지 (ROS 2 패키지)
│   ├── franka_ros2/        # Franka 공식 저장소 (gitignore, 별도 클론 필요)
│   └── franka_description/ # Franka 공식 저장소 (gitignore, 별도 클론 필요)
├── run_standalone.sh        # franka_ik_bridge 빌드+실행
└── README.md
```

## 공통 사전 요구사항

- Ubuntu 22.04

MuJoCo 3.4.0은 `mujoco-3.4.0/`에 이미 포함되어 있어 별도 설치가 필요 없습니다.

---

## A. `franka_ik_bridge` (ROS 2 + MoveIt 2 IK)

### 사전 요구사항

- ROS 2 Humble (아래에서 설치)
- MoveIt 2 (아래 rosdep 단계에서 함께 설치됨)

#### ROS 2 Humble 설치 (Ubuntu 22.04, Debian 패키지)

공식 가이드: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

```bash
# 1. UTF-8 로케일 확인 (보통 기본 설정되어 있음)
locale  # LANG=en_US.UTF-8 등으로 나오면 통과

# 2. ROS 2 apt 저장소 등록
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 설치
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base ros-dev-tools

# 4. rosdep 초기화 (최초 1회)
sudo rosdep init
rosdep update

# 5. 매 셸마다 자동 소싱하고 싶다면 (선택)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

`ros-humble-ros-base`는 최소 구성(RViz 등 GUI 도구 제외)이고, `ros-dev-tools`가 `colcon`, `rosdep` 등 빌드 도구를 포함합니다. MoveIt 2 자체는 여기서 설치하지 않고, 아래 `rosdep install` 단계에서 `franka_ik_bridge/package.xml`에 선언된 대로 자동 설치됩니다.

### 워크스페이스 준비

`src/franka_ros2`, `src/franka_description`는 Franka 공식 저장소로, 이 저장소에는 포함되어 있지 않습니다(`.gitignore` 처리). 클론 후 직접 받아야 합니다:

```bash
cd ~/franka_ws/src
git clone -b humble https://github.com/frankarobotics/franka_ros2.git
git clone https://github.com/frankaemika/franka_description.git
```

참고: 현재 개발은 아래 커밋 기준으로 진행되었습니다.
- `franka_ros2` (`humble` 브랜치): `7ef0ab0`
- `franka_description` (`main` 브랜치): `2b4c24c`

### 의존성 설치

```bash
cd ~/franka_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`franka_ik_bridge/package.xml`에 `moveit_core`, `moveit_ros_planning`, `moveit_ros_planning_interface`가 선언되어 있어 rosdep이 필요한 MoveIt 2 apt 패키지를 자동으로 설치합니다.

### 빌드 및 실행

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

> `run_standalone.sh`/`build_and_run.sh`는 `gnome-terminal`로 새 탭을 엽니다. Snap 환경 등에서 `libpthread.so.0: undefined symbol` 에러로 `gnome-terminal`이 안 뜨면, 스크립트의 `gnome-terminal --tab ...` 부분 대신 같은 셸에서 직접 `ros2 run franka_ik_bridge <노드명>`을 실행하세요.

---

## B. `previous_pd_project` (자코비안 전치 PD 제어, ROS 불필요)

### 사전 요구사항

ROS 2는 필요 없습니다. 아래만 있으면 됩니다.

```bash
sudo apt install g++ libglfw3-dev libgl1-mesa-dev
```

### 빌드 및 실행

```bash
cd ~/franka_ws/previous_pd_project
./build_and_run.sh
```

내부적으로 아래와 동일합니다 (직접 실행하고 싶을 때):

```bash
g++ -std=c++17 -O2 main.cpp \
    -I../mujoco-3.4.0/include -L../mujoco-3.4.0/lib \
    -lmujoco -lglfw -lGL -lpthread \
    -Wl,-rpath,'$ORIGIN/../mujoco-3.4.0/lib' \
    -o master_control
./master_control
```

`-Wl,-rpath` 덕분에 `LD_LIBRARY_PATH`를 따로 지정하지 않아도 `mujoco-3.4.0/lib`을 찾습니다. WASDQE로 타깃(빨간 구슬)을 움직이면 스프링-댐퍼 힘 제어(`F = Kp·e - Kd·ẋ`, 자코비안 전치로 토크 변환)로 팔이 따라갑니다. ESC로 종료.
