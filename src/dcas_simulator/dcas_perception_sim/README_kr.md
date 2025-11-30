# DCAS Perception Simulation (ROS1) — 사용 안내서

이 패키지는 4일 과정 DCAS 인지 시뮬레이션 교육을 위한 최소·완결형 프레임워크입니다.
환경(GT) 노드, 센서 에뮬레이터, 인지 스텁 노드, RViz 시각화 노드로 구성되어, 표준 메시지 위주로 빠르게 실습이 가능합니다.

## 요구 사항
- ROS1 Noetic(권장) / Python 3

## 빌드 및 환경설정
```bash
cd /home/ailab/git/NGV_DCAS_SW
catkin_make
source devel/setup.bash
```

## 실행
- 전체 노드 실행:
```bash
roslaunch dcas_perception_sim bringup.launch
```
- 시각화 포함 실행(RViz 자동 실행):
```bash
roslaunch dcas_perception_sim rviz.launch
```

### 차량 조작(키보드)
- 기본적으로 `vehicle_kinematic_node`와 `teleop_keyboard_node`가 함께 실행됩니다.
- 조작키: 방향키(좌/우 조향, 상 가속, 하 감속), Space 리셋, q 종료
- 비활성화하려면: `roslaunch dcas_perception_sim bringup.launch with_vehicle:=false`

## 주요 토픽
- 환경(GT): `/env/lanes`, `/env/traffic_lights`, `/env/guardrails`, `/env/buildings`, `/env/traffic_signs`, `/env/surrounding_vehicles`
- 센서: `/sensors/motion/*`, `/sensors/gnss/*`, `/sensors/imu/data`, `/sensors/lidar/points`, `/sensors/radar/points`, `/sensors/camera/image_raw`
- 인지: `/perception/*`
- 시각화: `/viz/markers` (`visualization_msgs/MarkerArray`)

## RViz 사용
- 고정 프레임: `map`
- 기본 디스플레이: `MarkerArray`(토픽 `/viz/markers`)
- 선택 추가: `PointCloud2`(LiDAR), `Image`(Camera)

---

## 노드별 설명
아래 노드들은 교육용 스텁/에뮬레이터로 간단한 데이터 흐름을 보여줍니다. 실제 알고리즘으로 교체하기 쉽도록 표준 메시지 위주로 구성했습니다.

### 환경 노드 (Ground Truth)
- `env_lane_node.py`
  - 기능: 간단한 곡률의 차선 중심/좌/우 경계를 생성
  - 퍼블리시: `/env/lanes` (`dcas_msgs/LaneArray`)
- `env_traffic_light_node.py`
  - 기능: 주기적으로 신호등(적/황/녹) 상태 토글
  - 퍼블리시: `/env/traffic_lights` (`dcas_msgs/TrafficLightArray`)
- `env_guardrail_node.py`
  - 기능: 일정 간격의 가드레일 박스 생성
  - 퍼블리시: `/env/guardrails` (`dcas_msgs/ObjectArray`)
- `env_building_node.py`
  - 기능: 도로 주변의 건물 박스 생성
  - 퍼블리시: `/env/buildings` (`dcas_msgs/ObjectArray`)
- `env_sign_node.py`
  - 기능: 단순 교통표지 객체 생성
  - 퍼블리시: `/env/traffic_signs` (`dcas_msgs/ObjectArray`)
- `env_surrounding_vehicle_node.py`
  - 기능: 반대 차선에서 접근하는 대항차를 시간에 따라 이동시켜 생성
  - 퍼블리시: `/env/surrounding_vehicles` (`dcas_msgs/ObjectArray`)

### 센서 노드 (Emulator)
- `sensor_motion_node.py`
  - 기능: 스티어링 휠 각도, 휠 스피드 펄스 간단 생성
  - 퍼블리시: `/sensors/motion/steering_wheel_angle` (`std_msgs/Float32`), `/sensors/motion/wheel_speed_pulses` (`std_msgs/Int32MultiArray`)
- `sensor_gnss_node.py`
  - 기능: NavSatFix, Odometry 간단 궤적 생성
  - 퍼블리시: `/sensors/gnss/fix` (`sensor_msgs/NavSatFix`), `/sensors/gnss/odom` (`nav_msgs/Odometry`)
- `sensor_imu_node.py`
  - 기능: 간단한 가속/각속도 신호 생성
  - 퍼블리시: `/sensors/imu/data` (`sensor_msgs/Imu`)
- `sensor_lidar_node.py`
  - 기능: 간단한 수평 라인 포인트클라우드 생성
  - 퍼블리시: `/sensors/lidar/points` (`sensor_msgs/PointCloud2`)
- `sensor_radar_node.py`
  - 기능: 접근하는 표적 1개의 도플러/거리 유사 값 생성
  - 퍼블리시: `/sensors/radar/points` (`sensor_msgs/PointCloud2`)
- `sensor_camera_node.py`
  - 기능: 단색(그린 채널) 더미 이미지 생성
  - 퍼블리시: `/sensors/camera/image_raw` (`sensor_msgs/Image`)

### 인지 노드
- 단순화 목적상 기본 패키지에서 제거했습니다. 필요 시 별도 패키지로 구현/실행하세요.

### 시각화 노드
환경 및 차량 노드가 각자 MarkerArray를 퍼블리시합니다.
  - 기능: GT/인지/레이더 토픽을 취합하여 `/viz/markers`(`visualization_msgs/MarkerArray`)로 변환
  - RViz에서 `MarkerArray` 디스플레이로 확인

---

## 디버깅/레코딩 팁
- 그래프 확인: `rqt_graph`
- 토픽 리스트/에코: `rostopic list`, `rostopic echo <topic>`
- 간단 레코딩:
```bash
rosbag record -O dcas_perception.bag /env/lanes /perception/lanes /viz/markers
```

## 확장 가이드
- 인지 스텁 노드를 실제 알고리즘으로 교체(입출력 토픽은 유지 권장)
- 에뮬레이터 파라미터(노이즈, 스폰 주기 등) 추가로 실험 다양화
- 필요 시 `dcas_msgs`에 메시지 확장(예: 차선 품질, 객체 확률 등)

## 라이선스
- Apache-2.0
