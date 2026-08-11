#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// MuJoCo 객체 선언
mjModel* m = nullptr;                  // 모델 구조체 (물리 파라미터 등)
mjData* d = nullptr;                   // 데이터 구조체 (현재 상태, 센서값 등)
mjvCamera cam;                         // 가상 카메라
mjvOption opt;                         // 시각화 옵션
mjvScene scn;                          // 3D 씬
mjrContext con;                        // 렌더링 컨텍스트

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// [핵심] 키보드 입력 콜백 함수
int active_joint = 0; // 현재 제어할 관절 인덱스 (0~6)

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS || act == GLFW_REPEAT) {
        
        // 1~7 숫자키를 눌러 제어할 관절 선택
        if (key >= GLFW_KEY_1 && key <= GLFW_KEY_7) {
            active_joint = key - GLFW_KEY_1; // '1'은 0번, '2'는 1번...
            printf("Selected Joint: %d\n", active_joint + 1);
        }

        // 선택된 관절을 방향키로 조종
        double step = 0.02; // 한 번 누를 때 움직일 각도(radian)
        
        if (key == GLFW_KEY_UP) {
            d->ctrl[active_joint] += step;
        }
        if (key == GLFW_KEY_DOWN) {
            d->ctrl[active_joint] -= step;
        }
    }
}

// 마우스 콜백 (카메라 조작용)
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    mjv_moveCamera(m, button_left ? mjMOUSE_ROTATE_V : (button_right ? mjMOUSE_MOVE_V : mjMOUSE_ZOOM),
                   dx/height, dy/height, &scn, &cam);
}

int main() {
    // 1. MuJoCo 모델 로드
    char error[1000];
    m = mj_loadXML("franka_fr3/fr3.xml", nullptr, error, 1000);
    if (!m) {
        printf("Error: %s\n", error);
        return 1;
    }
    d = mj_makeData(m);

    // 2. GLFW 초기화 및 윈도우 생성
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "FR3 Master Control", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 3. 시각화 데이터 구조체 초기화
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 4. 콜백 등록
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);

    // 5. 시뮬레이션 루프
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            mj_step(m, d);
        }

        // 렌더링
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 메모리 해제
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}
