#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>

class FrankaIKNode : public rclcpp::Node {
public:
    FrankaIKNode() : Node("franka_ik_node") {
        this->declare_parameter("robot_description", "");
        this->declare_parameter("robot_description_semantic", "");
        this->declare_parameter("base_path", "/home/leejoonhyoung/franka_ws/src/franka_ros2/franka_fr3_moveit_config/fr3/");
        
        this->declare_parameter("robot_description_kinematics.fr3_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.fr3_arm.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.fr3_arm.kinematics_solver_timeout", 0.05);
    }

    void init_model() {
        std::string base_path = this->get_parameter("base_path").as_string();
        auto read_file = [](const std::string& path) {
            std::ifstream ifs(path);
            if (!ifs.is_open()) return std::string("");
            std::stringstream ss; ss << ifs.rdbuf(); return ss.str();
        };

        std::string urdf = read_file(base_path + "fr3.urdf");
        std::string srdf = read_file(base_path + "fr3.srdf");

        if (urdf.empty() || srdf.empty()) {
            RCLCPP_ERROR(this->get_logger(), "파일 로드 실패!"); return;
        }

        // 1. URDF Mesh 경로 보정
        std::string pkg_path = "package://franka_fr3_moveit_config/fr3/";
        std::string abs_path = "file://" + base_path;
        size_t pos = 0;
        while ((pos = urdf.find(pkg_path, pos)) != std::string::npos) {
            urdf.replace(pos, pkg_path.length(), abs_path);
            pos += abs_path.length();
        }

        // 2. [수정] SRDF 로그 청소: 표준 정규표현식 사용
        // C++ std::regex에서 줄바꿈 포함 매칭은 [\s\S]*? 를 사용합니다.
        srdf = std::regex_replace(srdf, std::regex("<group name=\"fr3_hand\">[\\s\\S]*?</group>"), "");
        srdf = std::regex_replace(srdf, std::regex("<end_effector name=\"fr3_hand\"[\\s\\S]*?/>"), "");
        srdf = std::regex_replace(srdf, std::regex("<group_state name=\"(open|close)\"[\\s\\S]*?</group_state>"), "");
        
        // IK 팁 링크 보정 (TCP -> Link8)
        srdf = std::regex_replace(srdf, std::regex("fr3_hand_tcp"), "fr3_link8");

        this->set_parameter(rclcpp::Parameter("robot_description", urdf));
        this->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));

        auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "robot_description");
        kinematic_model_ = rml->getModel();
        kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        joint_model_group_ = kinematic_model_->getJointModelGroup("fr3_arm");
        tip_link_name_ = "fr3_link8";

        current_q_ = {0.0, -0.7853, 0.0, -2.3561, 0.0, 1.5707, 0.7853};
        
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                target_pose_ = *msg; has_target_ = true;
            });
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/mujoco_joint_commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&FrankaIKNode::solve_ik, this));
        
        RCLCPP_INFO(this->get_logger(), "T2 Solver Ready. Gripper parts removed from SRDF.");
    }

private:
    void solve_ik() {
        if (!has_target_) return;
        geometry_msgs::msg::Pose target = target_pose_.pose;
        kinematic_state_->setJointGroupPositions(joint_model_group_, current_q_);
        bool success = kinematic_state_->setFromIK(joint_model_group_, target, tip_link_name_, 0.02);
        if (success) {
            kinematic_state_->copyJointGroupPositions(joint_model_group_, current_q_);
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
            js.position = current_q_;
            pub_->publish(js);
        }
    }

    std::string tip_link_name_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped target_pose_;
    bool has_target_ = false;
    std::vector<double> current_q_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<FrankaIKNode>();
        node->init_model();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
