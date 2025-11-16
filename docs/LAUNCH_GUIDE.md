# LeRobot 통합 실행 가이드

모든 LeRobot 컴포넌트를 한 번에 실행하는 가이드입니다.

## 📋 시스템 구성 요소

1. **RealSense Camera (PointCloud)** - 3D 깊이 카메라 (Raw + Compressed)
2. **USB Camera** - 단안 RGB 카메라 (Raw + Compressed)
3. **Teleoperation Leader** - SO101 Leader (조종 암)
4. **Teleoperation Follower** - SO101 Follower (로봇 암)

모든 카메라는 **Raw**와 **Compressed** 형식 모두 발행합니다:
- Raw: 무손실 이미지 (대역폭 사용량 높음)
- Compressed: JPEG 압축 이미지 (대역폭 효율적, 약간의 품질 손실)

## 🚀 실행 방법

### 방법 1: 별도 터미널 창에서 실행 (권장)

각 컴포넌트가 별도의 터미널 창에서 실행되어 로그를 직접 볼 수 있습니다.

```bash
./launch_all.sh
```

**특징:**
- ✅ 각 컴포넌트의 로그를 실시간으로 확인 가능
- ✅ 개별 컴포넌트를 쉽게 재시작 가능
- ✅ 디버깅이 쉬움

**종료 방법:**
- 각 터미널 창에서 Ctrl+C
- 또는 터미널 창 닫기

---

### 방법 2: 백그라운드에서 실행

모든 컴포넌트가 백그라운드에서 실행되며, 로그는 파일로 저장됩니다.

```bash
./launch_all_background.sh
```

**특징:**
- ✅ 한 번에 모든 컴포넌트 실행
- ✅ 터미널 창 하나만 사용
- ✅ 로그 파일로 저장 (`log/` 디렉토리)

**로그 확인:**
```bash
# 실시간 로그 보기
tail -f log/realsense.log
tail -f log/usb_camera.log
tail -f log/leader.log
tail -f log/follower.log
```

**종료 방법:**
```bash
# Ctrl+C 누르거나
./stop_all.sh
```

---

## ⚙️ 설정 변경

### USB 카메라 설정 변경

환경 변수로 카메라 설정을 변경할 수 있습니다:

```bash
# 카메라 인덱스 변경 (기본: 0)
export USB_CAMERA_INDEX=1

# 해상도 변경 (기본: 640x480)
export USB_CAMERA_WIDTH=1920
export USB_CAMERA_HEIGHT=1080

# FPS 변경 (기본: 30)
export USB_CAMERA_FPS=60

# Topic 이름 변경 (기본: /camera/image_raw)
export USB_CAMERA_TOPIC="/my_camera/image"

# 설정 적용하여 실행
./launch_all_background.sh
```

**발행되는 Topic:**
- `${USB_CAMERA_TOPIC}` - Raw 이미지 (sensor_msgs/Image)
- `${USB_CAMERA_TOPIC}/compressed` - Compressed 이미지 (sensor_msgs/CompressedImage, JPEG 90% 품질)

### Teleoperation 포트 변경

`run_teleop_all.sh` 파일을 수정하세요:

```bash
# Leader 포트 (기본: /dev/ttyACM1)
--teleop.port=/dev/ttyUSB0

# Follower 포트 (기본: /dev/ttyACM0)
--robot.port=/dev/ttyUSB1
```

---

## 🔍 모니터링

### ROS2 Topic 확인

```bash
source /opt/ros/jazzy/setup.bash

# 모든 topic 목록
ros2 topic list

# 특정 topic의 메시지 보기
ros2 topic echo /camera/image_raw                      # USB 카메라 (Raw)
ros2 topic echo /camera/image_raw/compressed           # USB 카메라 (Compressed)
ros2 topic echo /camera/depth/image_rect_raw           # RealSense 깊이 (Raw)
ros2 topic echo /camera/depth/image_rect_raw/compressed # RealSense 깊이 (Compressed)
ros2 topic echo /camera/color/image_raw                # RealSense 컬러 (Raw)
ros2 topic echo /camera/color/image_raw/compressed     # RealSense 컬러 (Compressed)
ros2 topic echo /camera/depth/color/points             # RealSense PointCloud
ros2 topic echo /lerobot/leader/joint_states
ros2 topic echo /lerobot/follower/joint_states

# Topic 발행 빈도 확인
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_raw/compressed
ros2 topic hz /camera/depth/color/points
ros2 topic hz /lerobot/leader/joint_states
```

### ROS2 Node 확인

```bash
# 실행 중인 노드 목록
ros2 node list

# 노드 정보
ros2 node info /camera_publisher
ros2 node info /lerobot_leader_node
ros2 node info /lerobot_follower_node
```

### 시각화 (rviz2)

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

**rviz2 설정:**
1. **Add** 버튼 클릭
2. **By topic** 탭 선택
3. 원하는 topic 선택:
   - `/camera/image_raw` → Image (USB 카메라 - Raw)
   - `/camera/image_raw/compressed` → CompressedImage (USB 카메라 - Compressed)
   - `/camera/color/image_raw` → Image (RealSense 컬러 - Raw)
   - `/camera/color/image_raw/compressed` → CompressedImage (RealSense 컬러 - Compressed)
   - `/camera/depth/color/points` → PointCloud2 (RealSense PointCloud)
   - `/lerobot/leader/joint_states` → JointState

**Tip**: 네트워크 대역폭이 제한적인 경우 compressed topic을 사용하세요!

---

## 🛠️ 개별 컴포넌트 실행

필요한 컴포넌트만 개별적으로 실행할 수도 있습니다.

### RealSense 카메라만 실행

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_pointcloud_launch.py
```

### USB 카메라만 실행

```bash
./run_camera_ros2.sh
# 또는 설정 변경
./run_camera_ros2.sh 0 /camera/image_raw 1920 1080 30
```

**파라미터:**
1. Camera index (기본: 0)
2. Topic name (기본: /camera/image_raw)
3. Width (기본: 640)
4. Height (기본: 480)
5. FPS (기본: 30)

### Teleoperation만 실행

```bash
./run_teleop_all.sh
```

---

## 🔧 문제 해결

### 카메라를 찾을 수 없음

```bash
# 사용 가능한 카메라 확인
ls -la /dev/video*

# LeRobot 카메라 찾기 도구
conda activate lerobot
lerobot-find-cameras
```

### 포트를 찾을 수 없음

```bash
# 사용 가능한 포트 확인
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*

# LeRobot 포트 찾기 도구
conda activate lerobot
lerobot-find-port
```

### 포트 권한 에러

```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -aG dialout $USER

# 로그아웃 후 다시 로그인 필요
```

### 프로세스가 종료되지 않음

```bash
# 강제 종료
./stop_all.sh

# 또는 수동으로 종료
pkill -f "run_camera_ros2"
pkill -f "run_teleoperate_ros2"
pkill -f "realsense2_camera"
```

### ROS2를 찾을 수 없음

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# 또는 conda 환경 사용
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash
```

---

## 📊 시스템 요구사항

- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy
- **Python**: 3.12 (lerobot_ros2 conda 환경)
- **하드웨어**:
  - Intel RealSense Camera (D435, D455 등)
  - USB 카메라
  - SO101 Leader 암 (USB 또는 Serial)
  - SO101 Follower 암 (USB 또는 Serial)

---

## 📁 파일 구조

```
lerobot/
├── launch_all.sh                       # 별도 터미널에서 모든 컴포넌트 실행
├── launch_all_background.sh            # 백그라운드에서 모든 컴포넌트 실행
├── stop_all.sh                         # 모든 컴포넌트 종료
├── run_camera_ros2.sh                 # USB 카메라 Publisher
├── run_teleop_all.sh                  # Teleoperation (Leader + Follower)
├── run_teleoperate_ros2.sh            # 개별 Teleoperation 노드
├── calibrate_leader.sh                # Leader Calibration
├── calibrate_follower.sh              # Follower Calibration
├── ros2_bag_recorder_gui.py           # ROS 2 Bag 녹화 GUI (기본)
├── ros2_bag_recorder_gui_v2.py        # ROS 2 Bag 녹화 GUI (메타데이터 수집 ⭐)
├── run_bag_recorder_gui.sh            # 기본 GUI 실행 스크립트
├── run_bag_recorder_with_metadata.sh  # 메타데이터 수집 GUI 실행 스크립트
├── METADATA_COLLECTION_GUIDE.md       # 메타데이터 수집 가이드
├── BAG_RECORDER_GUI_GUIDE.md          # 기본 GUI 가이드
├── data/                              # 녹화된 Rosbag 파일들
│   └── 20251116_175800_pick_place_red_cube/
│       ├── metadata.yaml
│       └── 20251116_175800_pick_place_red_cube_0.db3
├── 20251116_175800_pick_place_red_cube.json  # 메타데이터 JSON (v2 GUI 사용 시)
└── log/                               # 로그 디렉토리 (백그라운드 모드)
    ├── realsense.log
    ├── realsense_color_compressed.log
    ├── realsense_depth_compressed.log
    ├── usb_camera.log
    ├── leader.log
    └── follower.log
```

---

## 🎯 빠른 시작

### 1. Calibration (최초 1회)

```bash
# Leader calibration
./calibrate_leader.sh

# Follower calibration
./calibrate_follower.sh
```

### 2. 모든 컴포넌트 실행

```bash
# 별도 터미널에서 (권장)
./launch_all.sh

# 또는 백그라운드에서
./launch_all_background.sh
```

### 3. 작동 확인

```bash
source /opt/ros/jazzy/setup.bash

# Topic 확인
ros2 topic list

# 시각화
rviz2
```

### 4. 종료

```bash
# 각 터미널에서 Ctrl+C
# 또는
./stop_all.sh
```

---

## 🎥 데이터 녹화 (ROS 2 Bag)

시스템이 실행 중일 때 데이터를 녹화할 수 있습니다.

### 방법 1: 메타데이터 수집 GUI (VLA/RFM 학습용 - 권장 ⭐)

VLA(Vision-Language-Action) 및 RFM(Robotics Foundation Model) 학습을 위한 **메타데이터 수집 기능**이 포함된 GUI입니다:

```bash
./run_bag_recorder_with_metadata.sh
```

**주요 기능:**
- 2개 카메라 실시간 프리뷰 (Head + Wrist)
- **토픽 선택 기능**: 현재 발행 중인 토픽 중에서 선택하여 녹화
  - 기본 4개 토픽 자동 선택 (카메라 + 관절 상태)
  - 🔄 Refresh Topics 버튼으로 실시간 토픽 목록 갱신
  - 체크박스로 간편하게 선택/해제
- **메타데이터 수집 패널**:
  - Task Information (지시어, 작업명, 작업 유형, 태그)
  - Collection Context (작업자, 로봇 모델, 환경 정보)
  - Hardware Configuration (카메라명, 그리퍼 모델)
  - Recording Topics (녹화할 토픽 선택)
  - Custom Fields (성공 여부, 실패 이유)
- **자동 JSON 생성**: 녹화 종료 시 메타데이터 JSON 파일 자동 생성 (Schema v1.0.0)
  - 선택된 토픽의 타입 정보 자동 수집
- **1:1 파일 매핑**: Rosbag과 동일한 base name으로 JSON 저장
- 키보드 단축키 (A/S/D)

**사용 예시:**
1. GUI 실행 후 우측 패널에서 메타데이터 입력
   - Task ID: `pick_and_place_red_cube_001`
   - Instruction: `"Place the red cube into the blue bowl."`
   - Operator: `junmo`
2. 'A' 키로 녹화 시작 → 작업 수행 → 'S' 키로 저장
3. 생성 파일:
   - `data/20251116_175800_pick_place_red_cube/` (Rosbag)
   - `20251116_175800_pick_place_red_cube.json` (Metadata)

**자세한 사용법**: [METADATA_COLLECTION_GUIDE.md](METADATA_COLLECTION_GUIDE.md)

---

### 방법 2: 기본 GUI (단순 녹화)

실시간 카메라 프리뷰와 함께 bag 파일만 녹화합니다 (메타데이터 수집 없음):

```bash
./run_bag_recorder_gui.sh
```

**기능:**
- 2개 카메라 실시간 프리뷰:
  - Head View: `/camera/camera/color/image_raw/compressed` (RealSense)
  - Wrist View: `/camera/image_raw/compressed` (USB)
- 4개 토픽 자동 녹화:
  - `/camera/color/image_raw/compressed` (RealSense 컬러)
  - `/camera/depth/color/points` (RealSense PointCloud)
  - `/camera/image_raw/compressed` (USB 카메라)
  - `/lerobot/follower/joint_states` (로봇 관절 상태)
- 시작/저장/취소 버튼 + 키보드 단축키 (A/S/D)
- 녹화 시간 표시

**자세한 사용법**: [BAG_RECORDER_GUI_GUIDE.md](BAG_RECORDER_GUI_GUIDE.md)

---

### 방법 3: 명령어로 녹화

```bash
source /opt/ros/jazzy/setup.bash

# 녹화 시작
ros2 bag record \
  /camera/color/image_raw/compressed \
  /camera/depth/color/points \
  /camera/image_raw/compressed \
  /lerobot/follower/joint_states \
  -o my_recording

# Ctrl+C로 녹화 종료

# 재생
ros2 bag play my_recording

# 정보 확인
ros2 bag info my_recording
```

---

## 📚 관련 문서

- [METADATA_COLLECTION_GUIDE.md](METADATA_COLLECTION_GUIDE.md) - **메타데이터 수집 가이드 (VLA/RFM 학습용 ⭐)**
- [BAG_RECORDER_GUI_GUIDE.md](BAG_RECORDER_GUI_GUIDE.md) - ROS 2 Bag 녹화 GUI 가이드 (기본)
- [ROS2_QUICK_START.md](ROS2_QUICK_START.md) - ROS2 빠른 시작 가이드
- [ROS2_INTEGRATION.md](ROS2_INTEGRATION.md) - ROS2 통합 상세 가이드
- [CLAUDE.md](CLAUDE.md) - LeRobot 전체 문서

---

## ⚡ 추가 팁

### 자동 실행 설정

시스템 부팅 시 자동으로 실행하려면:

```bash
# systemd 서비스 생성
sudo nano /etc/systemd/system/lerobot.service
```

```ini
[Unit]
Description=LeRobot System
After=network.target

[Service]
Type=forking
User=weed
WorkingDirectory=/home/weed/lerobot
ExecStart=/home/weed/lerobot/launch_all_background.sh
ExecStop=/home/weed/lerobot/stop_all.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
# 서비스 활성화
sudo systemctl enable lerobot
sudo systemctl start lerobot

# 상태 확인
sudo systemctl status lerobot
```

### 성능 최적화

CPU 사용률이 높을 경우:

```bash
# FPS 낮추기
export USB_CAMERA_FPS=15

# 해상도 낮추기
export USB_CAMERA_WIDTH=320
export USB_CAMERA_HEIGHT=240

./launch_all_background.sh
```

### Compressed vs Raw Topic 사용 가이드

**Raw Topic 사용 권장:**
- 로컬에서 직접 처리
- 최고 화질이 필요한 경우
- 대역폭 제약이 없는 경우

**Compressed Topic 사용 권장:**
- 네트워크로 전송하는 경우
- 여러 노드가 구독하는 경우
- 대역폭이 제한적인 경우
- 디스크 공간을 절약하고 싶은 경우

**대역폭 비교 (640x480 @ 30fps 기준):**
- Raw: ~27 MB/s (640 × 480 × 3 bytes × 30 fps)
- Compressed (JPEG 90%): ~2-5 MB/s (약 80-90% 절감)
