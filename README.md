# Digital Twin SW Practice

Digital Twin를 위한 연습용 저장소로, ROS 노드 기초, Lanelet2 활용, 센서 시뮬레이션, 실제 센서 데이터 분석을 단계적으로 다룹니다.

## 저장소 구조 (Repository Structure)

```
NGV_DCAS_SW_Practice/
├─ README.md
├─ requirements.txt                # 노트북/데이터 분석용 Python 의존성
└─ src/
   ├─ ros_tutorial/                # 최소 ROS 예제(노드, 메시지, 런치)
   │  ├─ launch/
   │  │  └─ tutorial_demo.launch
   │  ├─ msg/                      # TutorialSineWaveInput, TutorialVehicleState
   │  └─ scripts/                  # 예제 노드(연습용 불완전 코드 포함)
   ├─ lanelet_tutorial/            # Lanelet2 사용 예제 및 샘플 맵
   │  ├─ README_Lanelet.md
   │  └─ lanelet2_maps/
   ├─ dcas_simulator/              # 센서 시뮬레이터 예제
   │  ├─ dcas_msgs/                # 시뮬레이터용 메시지/패키지 스텁
   │  └─ dcas_perception_sim/      # 지각(Perception) 시뮬레이션 예제
   └─ sensor_data_analysis/        # 실제 센서 데이터 처리(Jupyter)
      ├─ data/
      ├─ resources/
      └─ practice/
         ├─ 1_dev_environment.ipynb
         ├─ 2_motion_sensor.ipynb
         ├─ 3_gnss.ipynb
         ├─ 4_lidar.ipynb
         ├─ 5_radar.ipynb
         └─ 6_camera.ipynb
```

## 구성 설명 (Components)

### 1) ros_tutorial — 최소 ROS 노드 예제
노드 생성, 파라미터 처리, 퍼블리셔/서브스크라이버, TF, 시각화를 학습하기 위한 간단한 예제입니다.

- 노드(연습 지향; 일부는 `xxxxxxxxx` 형태의 빈칸이 남아 있음):
  - `sine_wave_set_node.py`: 사인파 파라미터(`period`, `amplitude`, `noise_standard_deviation`) 퍼블리시
  - `steering_sine_wave_generator_node.py`: 파라미터를 구독하여 노이즈가 포함된 조향 명령(`/steering_cmd`) 출력
  - `kinematic_bicycle_model_node.py`: 조향 명령을 받아 자전거 모델 시뮬레이션 수행. 차량 상태/TF/Marker 퍼블리시
- 런치: `src/ros_tutorial/launch/tutorial_demo.launch`
  - 세 노드를 인자와 함께 연결합니다.

주요 토픽:
- 입력: `/sine_wave/input` (커스텀 메시지 `TutorialSineWaveInput`)
- 출력: `/steering_cmd` (표준 메시지 `std_msgs/Float64`)
- 상태/시각화: `/vehicle/state`, `/vehicle/marker`, TF 프레임 `map -> vehicle`

### 2) lanelet_tutorial — Lanelet2 사용 예제
Lanelet2 설치 및 활용, 샘플 맵 데이터 사용 예제를 포함합니다.
- ROS Noetic에서 빠른 설치:
  ```bash
  sudo apt install ros-noetic-lanelet2
  ```
- 추가 내용: `src/lanelet_tutorial/README_Lanelet.md`

### 3) sensor_data_analysis — 실제 센서 데이터 처리
IMU, GNSS, LiDAR, RADAR, Camera 등 실제 센서 데이터를 파싱/변환/시각화하는 Jupyter 노트북 모음입니다.
- 노트북: `practice/1_dev_environment.ipynb` … `practice/6_camera.ipynb`
- 보조 자료: `data/`, `resources/`

### 4) dcas_simulator — 센서 시뮬레이터 예제
Perception 파이프라인과 메시지 시뮬레이션을 위한 스캐폴딩입니다.
- `dcas_msgs/`: 시뮬레이터 예제를 위한 메시지/패키지 정의
- `dcas_perception_sim/`: Perception 시뮬레이션용 예제 노드와 유틸리티

## 사전 준비 (Prerequisites)

- OS: Ubuntu 20.04 LTS (ROS Noetic 권장)
- ROS: Noetic (desktop-full) — ROS 구성 요소용
- Python: 3.8 — 노트북/스크립트용

ROS 설치(미설치 시):
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full python3-rosdep
sudo rosdep init || true
rosdep update
```

Lanelet2 설치(lanelet_tutorial용):
```bash
sudo apt install ros-noetic-lanelet2
```

Python 의존성 설치(데이터 분석/노트북용):
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 빌드 및 실행 (ROS)

이 저장소는 `src/` 하위에 ROS 패키지를 포함합니다. Catkin 워크스페이스 안에 배치하는 방법을 권장합니다.

```bash
# Catkin 워크스페이스 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 저장소 클론
git clone https://github.com/ailab-hanyang/digital_twin_sw.git

# 빌드
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

연습용 런치 실행(연습용 빈칸을 먼저 채운 뒤):
```bash
roslaunch ros_tutorial tutorial_demo.launch
```

참고:
- 일부 튜토리얼 노드는 학습용으로 `xxxxxxxxx`가 남아 있습니다. 실행 전 완성하세요.
- 워크스페이스를 소싱한 후(`source devel/setup.bash`) 패키지 인식 확인:
  ```bash
  rospack list | grep ros_tutorial
  ```

## 노트북 실행 (Jupyter)

```bash
# 위에서 만든 가상환경 활성화(선택)
source .venv/bin/activate

# Jupyter 실행
jupyter notebook
# 또는
jupyter lab
```
`src/sensor_data_analysis/practice/`의 노트북을 순서대로 실행하세요.

## Contributing & Notes

- 많은 예제가 연습을 위해 설계되었습니다. 주석/빈칸을 채워보세요.
- ROS와 Python 의존성을 분리 유지(Catkin 워크스페이스 vs. Python venv).

## Option - vehicle model 시각화 in WSL2
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3COMPAT
export MESA_GLSL_VERSION_OVERRIDE=330
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export PULSE_SERVER=unix:/mnt/wslg/PulseServer
