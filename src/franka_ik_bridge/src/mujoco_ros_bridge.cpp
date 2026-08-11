#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>

// --- 전역 변수 및 MuJoCo 객체 ---
mjModel* m = nullptr; mjData* d = nullptr;
mjvCamera cam; mjvOption opt; mjvScene scn; mjrContext con;
GLFWwindow* window = nullptr;

bool bl = false; bool br = false; double lx, ly;
std::vector<double> t_q = {0.0, -0.7853, 0.0, -2.3561, 0.0, 1.5707, 0.7853};
double px = 0.5, py = 0.0, pz = 0.5;
double roll = 3.1415, pitch = 0.0, yaw = 0.0; // 초기 상태: 지면 하향

// --- GLFW 콜백 함수 ---
void mouse_button(GLFWwindow* w, int b, int a, int mod) {
    bl = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    br = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(w, &lx, &ly);
}

void mouse_move(GLFWwindow* w, double x, double y) {
    if (!bl && !br) return;
    double dx = x - lx; double dy = y - ly; lx = x; ly = y;
    int width, height; glfwGetWindowSize(w, &width, &height);
    mjv_moveCamera(m, br ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V, dx/width, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* w, double x, double y) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*y, &scn, &cam);
}

// --- ROS 2 노드 클래스 ---
class MujocoBridge : public rclcpp::Node {
public:
    MujocoBridge() : Node("mujoco_bridge_node") {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/mujoco_joint_commands", 10, 
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if(msg->position.size() >= 7) {
                    for(int i=0; i<7; i++) t_q[i] = msg->position[i];
                }
            });

        char err[1000];
        std::string xml = "/home/leejoonhyoung/franka_ws/src/franka_ros2/franka_fr3_moveit_config/fr3/fr3_scene.xml";
        m = mj_loadXML(xml.c_str(), 0, err, 1000);
        if (!m) { RCLCPP_ERROR(this->get_logger(), "XML 로드 실패: %s", err); exit(1); }
        d = mj_makeData(m);

        glfwInit();
        window = glfwCreateWindow(1200, 900, "T3 6-DOF Mujoco Bridge", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetScrollCallback(window, scroll);

        mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 2000); mjr_makeContext(m, &con, mjFONTSCALE_150);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MujocoBridge::loop, this));
    }

private:
    void loop() {
        if (glfwWindowShouldClose(window)) return;

        // 조작키: 위치(WASDQE), 회전(UIOJKL)
        if(glfwGetKey(window, GLFW_KEY_W)==GLFW_PRESS) px+=0.005; if(glfwGetKey(window, GLFW_KEY_S)==GLFW_PRESS) px-=0.005;
        if(glfwGetKey(window, GLFW_KEY_A)==GLFW_PRESS) py+=0.005; if(glfwGetKey(window, GLFW_KEY_D)==GLFW_PRESS) py-=0.005;
        if(glfwGetKey(window, GLFW_KEY_Q)==GLFW_PRESS) pz+=0.005; if(glfwGetKey(window, GLFW_KEY_E)==GLFW_PRESS) pz-=0.005;
        if(glfwGetKey(window, GLFW_KEY_U)==GLFW_PRESS) roll+=0.03; if(glfwGetKey(window, GLFW_KEY_J)==GLFW_PRESS) roll-=0.03;
        if(glfwGetKey(window, GLFW_KEY_I)==GLFW_PRESS) pitch+=0.03; if(glfwGetKey(window, GLFW_KEY_K)==GLFW_PRESS) pitch-=0.03;
        if(glfwGetKey(window, GLFW_KEY_O)==GLFW_PRESS) yaw+=0.03; if(glfwGetKey(window, GLFW_KEY_L)==GLFW_PRESS) yaw-=0.03;

        // Euler to Quat (회전 반영)
        double cr = cos(roll*0.5); double sr = sin(roll*0.5);
        double cp = cos(pitch*0.5); double sp = sin(pitch*0.5);
        double cy = cos(yaw*0.5); double sy = sin(yaw*0.5);
        double qw = cr*cp*cy + sr*sp*sy; double qx = sr*cp*cy - cr*sp*sy;
        double qy = cr*sp*cy + sr*cp*sy; double qz = cr*cp*sy - sr*sp*cy;

        int bid = mj_name2id(m, mjOBJ_BODY, "target_ball");
        if (bid != -1) {
            int mid = m->body_mocapid[bid];
            d->mocap_pos[mid*3+0] = px; d->mocap_pos[mid*3+1] = py; d->mocap_pos[mid*3+2] = pz;
            d->mocap_quat[mid*4+0] = qw; d->mocap_quat[mid*4+1] = qx; d->mocap_quat[mid*4+2] = qy; d->mocap_quat[mid*4+3] = qz;
        }

        for(int i=0; i<7; i++) d->ctrl[i] = t_q[i];
        mj_step(m, d);

        auto ps = geometry_msgs::msg::PoseStamped();
        ps.header.stamp = this->now();
        ps.pose.position.x = px; ps.pose.position.y = py; ps.pose.position.z = pz;
        ps.pose.orientation.w = qw; ps.pose.orientation.x = qx; ps.pose.orientation.y = qy; ps.pose.orientation.z = qz;
        pub_->publish(ps);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjrRect vp = {0, 0, 0, 0}; glfwGetFramebufferSize(window, &vp.width, &vp.height);
        mjr_render(vp, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 멤버 변수 선언 (이 부분이 누락되었었습니다)
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoBridge>());
    rclcpp::shutdown();
    return 0;
}
