#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <cmath>

// --- 전역 변수 ---
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam; mjvOption opt; mjvScene scn; mjrContext con;
std::mutex mtx;

bool running = true;
double target_pos[3] = {0.4, 0.0, 0.5}; 
double lastx = 0, lasty = 0;
bool button_left = false, button_right = false;

// --- 제어 게인 (안정성 중심) ---
// 이 수식에서는 k_p가 스프링 상수, k_d가 댐핑 상수 역할을 합니다.
double k_p = 220.0; 
double k_d = 50.0;  

// --- 키보드/마우스 콜백 (이전과 동일) ---
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS || act == GLFW_REPEAT) {
        std::lock_guard<std::mutex> lock(mtx);
        switch (key) {
            case GLFW_KEY_W: target_pos[0] += 0.02; break;
            case GLFW_KEY_S: target_pos[0] -= 0.02; break;
            case GLFW_KEY_A: target_pos[1] += 0.02; break;
            case GLFW_KEY_D: target_pos[1] -= 0.02; break;
            case GLFW_KEY_Q: target_pos[2] += 0.02; break;
            case GLFW_KEY_E: target_pos[2] -= 0.02; break;
            case GLFW_KEY_ESCAPE: running = false; break;
        }
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_right) return;
    double dx = xpos - lastx; double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int width, height; glfwGetWindowSize(window, &width, &height);
    mjtMouse action = button_left ? mjMOUSE_ROTATE_V : mjMOUSE_MOVE_V;
    std::lock_guard<std::mutex> lock(mtx);
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// --- 물리 및 제어 루프 ---
void physics_thread() {
    int ee_id = mj_name2id(m, mjOBJ_SITE, "attachment_site");
    int nv = m->nv;

    while (running) {
        {
            std::lock_guard<std::mutex> lock(mtx);

            // 1. 자코비안 획득 (3 x 7)
            mjtNum jacp[3 * 7]; 
            mj_jacSite(m, d, jacp, NULL, ee_id);

            // 2. 엔드이펙터의 실제 속도(x_dot) 획득
            mjtNum site_vel[6]; 
            mj_objectVelocity(m, d, mjOBJ_SITE, ee_id, site_vel, 0);

            // 3. 작업 공간 가상 힘 계산 (F = Kp*e - Kd*x_dot)
            // 에러(e)와 속도(x_dot)의 부호가 반대여야 감쇠(Damping)가 일어납니다.
            mjtNum force[3];
            for (int i = 0; i < 3; i++) {
                double error = target_pos[i] - d->site_xpos[ee_id * 3 + i];
                // 에너지를 공급하는 Positive Feedback 방지를 위해 부호 엄격 준수
                force[i] = k_p * error - k_d * site_vel[i];
            }

            // 4. 자코비안 전치를 이용한 토크 변환 (tau = J^T * F)
            mjtNum tau[7];
            mju_mulMatTVec(tau, jacp, force, 3, nv);

            // 5. 최종 토크 인가 (중력 보상 + 추가 관절 댐핑)
            for (int i = 0; i < nv; i++) {
                // 발산을 막기 위한 최후의 보루: 관절 공간 물리적 댐핑
                double joint_damping = -5.0 * d->qvel[i]; 
                
                d->ctrl[i] = tau[i] + d->qfrc_bias[i] + joint_damping;
                
                // 시스템 보호를 위한 토크 리미트
                d->ctrl[i] = std::max(-80.0, std::min(80.0, d->ctrl[i]));
            }

            mj_step(m, d);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
}

int main() {
    m = mj_loadXML("franka_fr3/fr3.xml", nullptr, nullptr, 0);
    if (!m) { std::cerr << "모델 로드 실패!" << std::endl; return 1; }
    
    m->opt.timestep = 0.002;
    d = mj_makeData(m);

    // 초기 자세 설정 (안정적인 굽힘 자세)
    d->qpos[1] = -0.785; d->qpos[3] = -2.356; d->qpos[5] = 1.571;
    mj_forward(m, d);

    // 타겟 위치 동기화
    int ee_id = mj_name2id(m, mjOBJ_SITE, "attachment_site");
    for(int i=0; i<3; i++) target_pos[i] = d->site_xpos[ee_id * 3 + i];

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Stable Jacobian Transpose Control", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);

    mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjrContext con;
    mjr_defaultContext(&con); mjv_makeScene(m, &scn, 2000); mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.distance = 2.5; cam.lookat[0] = 0.4; cam.lookat[2] = 0.4;
    std::thread phys_thread(physics_thread);

    while (!glfwWindowShouldClose(window) && running) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            int w, h; glfwGetFramebufferSize(window, &w, &h);
            mjrRect v = {0, 0, w, h};
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            if (scn.ngeom < scn.maxgeom) {
                mjvGeom* g = &scn.geoms[scn.ngeom++];
                mjtNum size[3] = {0.02, 0, 0}; float color[4] = {1, 0, 0, 1};
                mjv_initGeom(g, mjGEOM_SPHERE, size, target_pos, NULL, color);
            }
            mjr_render(v, &scn, &con);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    running = false;
    phys_thread.join();
    mj_deleteData(d); mj_deleteModel(m);
    mjr_freeContext(&con); mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}
