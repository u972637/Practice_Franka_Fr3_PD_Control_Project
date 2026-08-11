#!/bin/bash
# previous_pd_project 빌드 & 실행 스크립트 (colcon 아님, g++ 직접 컴파일)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MUJOCO_DIR="$SCRIPT_DIR/../mujoco-3.4.0"
cd "$SCRIPT_DIR"

echo "빌드를 시작합니다..."
g++ -std=c++17 -O2 main.cpp \
    -I"$MUJOCO_DIR/include" \
    -L"$MUJOCO_DIR/lib" \
    -lmujoco -lglfw -lGL -lpthread \
    -Wl,-rpath,'$ORIGIN/../mujoco-3.4.0/lib' \
    -o master_control

if [ $? -ne 0 ]; then
    echo "❌ 빌드 실패! 위 에러 메시지를 확인하세요."
    read -p "엔터를 누르면 종료됩니다..."
    exit 1
fi

echo "빌드 완료. 실행합니다..."
pkill -f "$SCRIPT_DIR/master_control" 2>/dev/null || true
gnome-terminal --tab --title="Previous_PD_Project" -- bash -c "cd '$SCRIPT_DIR'; ./master_control; exec bash"

echo "완료. MuJoCo 창에서 WASDQE로 조종하세요."
