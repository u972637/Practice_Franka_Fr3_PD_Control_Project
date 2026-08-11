import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # 1. 경로 설정
    moveit_config_path = get_package_share_directory("franka_fr3_moveit_config")
    
    # 2. SRDF 로드 (로봇 그룹 정보 등)
    srdf_file = os.path.join(moveit_config_path, "config", "fr3.srdf")
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()

    # 3. Kinematics 설정 로드 (IK 솔버 파라미터)
    kin_file = os.path.join(moveit_config_path, "config", "kinematics.yaml")
    with open(kin_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # 4. IK 노드 실행 설정
    return LaunchDescription([
        Node(
            package='franka_ik_bridge',
            executable='franka_ik_node',
            output='screen',
            parameters=[
                {
                    'robot_description_semantic': semantic_content,
                    'use_sim_time': True,
                },
                kinematics_config,
            ],
        )
    ])
