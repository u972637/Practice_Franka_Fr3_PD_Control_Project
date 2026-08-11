cmake_minimum_required(VERSION 3.22)
project(franka_ik_bridge)

# 1. 환경 설정: MuJoCo 설치 경로 (버전 숫자 주의: 3.4.0 확인)
set(MUJOCO_DIR /home/leejoonhyoung/mujoco-3.4.0)

# 2. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenGL REQUIRED)

# [필수] MoveIt 핵심 구성 요소들
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 3. 헤더 경로 설정
include_directories(include)

# 4. 실행 파일 1: 통합 브리지 노드 (MuJoCo + ROS 2)
add_executable(mujoco_ros_bridge src/mujoco_ros_bridge.cpp)
target_include_directories(mujoco_ros_bridge PRIVATE ${MUJOCO_DIR}/include)
target_link_libraries(mujoco_ros_bridge 
    ${MUJOCO_DIR}/lib/libmujoco.so 
    glfw
    GL
)
ament_target_dependencies(mujoco_ros_bridge
    rclcpp
    geometry_msgs
    sensor_msgs
)

# 5. 실행 파일 2: IK 계산 노드 (T2)
add_executable(franka_ik_node src/franka_ik_node.cpp)
# MoveIt 라이브러리들을 명시적으로 링크합니다.
ament_target_dependencies(franka_ik_node
    rclcpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    geometry_msgs
    sensor_msgs
)

# 6. 설치 설정
install(TARGETS mujoco_ros_bridge franka_ik_node 
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
