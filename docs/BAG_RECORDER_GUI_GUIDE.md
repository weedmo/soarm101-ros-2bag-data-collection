# ROS 2 Bag Recorder GUI 사용 가이드

ROS 2 bag 파일을 녹화하고 카메라 영상을 실시간으로 확인할 수 있는 GUI 애플리케이션입니다.

## 📋 기능

### 실시간 카메라 뷰
- **Head View**: `/camera/camera/color/image_raw/compressed` (RealSense 컬러 카메라)
- **Wrist View**: `/camera/image_raw/compressed` (USB 카메라)

### 자동 녹화 토픽 (4개)
1. `/camera/color/image_raw/compressed` - RealSense 컬러 이미지 (Compressed)
2. `/camera/depth/color/points` - RealSense PointCloud
3. `/camera/image_raw/compressed` - USB 카메라 이미지 (Compressed)
4. `/lerobot/follower/joint_states` - 로봇 관절 상태

## 🚀 실행 방법

### 1. 시스템 준비

먼저 모든 컴포넌트가 실행 중인지 확인하세요:

```bash
# 모든 컴포넌트 실행 (별도 터미널)
./launch_all.sh

# 또는 백그라운드 실행
./launch_all_background.sh
```

다음 토픽들이 발행되고 있어야 합니다:
- `/camera/camera/color/image_raw/compressed` ✓
- `/camera/image_raw/compressed` ✓
- `/camera/depth/color/points` ✓
- `/lerobot/follower/joint_states` ✓

### 2. GUI 실행

```bash
./run_bag_recorder_gui.sh
```

## 🎮 GUI 사용법

### 화면 구성

```
┌─────────────────────────────────────────────────────────┐
│                  Head View    │    Wrist View           │
│             (RealSense 컬러)   │   (USB 카메라)           │
├─────────────────────────────────────────────────────────┤
│              Recording Time: 00:00                      │
│  [ 시작 [A] ]  [ 저장 [S] ]  [ 취소 [D] ]               │
└─────────────────────────────────────────────────────────┘
│ 상태: 대기 중 (Ready)                                    │
└─────────────────────────────────────────────────────────┘
```

### 키보드 단축키
- **A 키**: 녹화 시작
- **S 키**: 녹화 저장
- **D 키**: 녹화 취소

### 버튼 설명

#### 1. 시작 (Start) 버튼 또는 A 키 - 녹화 시작
- 현재 시간을 기준으로 bag 파일 생성 (예: `data_20251116_173045`)
- 4개 토픽의 녹화 시작
- 타이머 시작 (00:00부터 카운트)
- "저장" 및 "취소" 버튼 활성화

**파일 저장 위치**: `~/lerobot/data_YYYYMMDD_HHMMSS/`

**단축키**: **A 키**

#### 2. 저장 (Save) 버튼 또는 S 키 - 녹화 종료 및 저장
- 녹화 중지
- bag 파일 저장
- 타이머 리셋
- 상태: "저장 완료 (Saved to data_YYYYMMDD_HHMMSS)"

**중요**: bag 파일은 디렉터리 형태로 저장됩니다:
```
~/lerobot/data_20251116_173045/
  ├── metadata.yaml
  └── data_20251116_173045_0.db3
```

**단축키**: **S 키**

#### 3. 취소 (Cancel) 버튼 또는 D 키 - 녹화 취소
- 녹화 중지
- **bag 파일 완전 삭제**
- 타이머 리셋
- 상태: "녹화 취소됨 (Recording cancelled)"

**주의**: 취소하면 녹화된 데이터가 영구 삭제됩니다!

**단축키**: **D 키**

## 📊 녹화 데이터 확인

### bag 파일 정보 확인

```bash
cd ~/lerobot

# bag 파일 정보 보기
ros2 bag info data_20251116_173045

# 출력 예시:
# Files:             data_20251116_173045_0.db3
# Bag size:          123.4 MB
# Storage id:        sqlite3
# Duration:          45.2s
# Start:             Nov 16 2025 17:30:45.123
# End:               Nov 16 2025 17:31:30.456
# Messages:          1234
# Topic information:
#   /camera/color/image_raw/compressed | sensor_msgs/msg/CompressedImage | 300
#   /camera/depth/color/points | sensor_msgs/msg/PointCloud2 | 300
#   /camera/image_raw/compressed | sensor_msgs/msg/CompressedImage | 300
#   /lerobot/follower/joint_states | sensor_msgs/msg/JointState | 334
```

### bag 파일 재생

```bash
# 재생
ros2 bag play data_20251116_173045

# 특정 속도로 재생 (0.5배속)
ros2 bag play data_20251116_173045 --rate 0.5

# 특정 토픽만 재생
ros2 bag play data_20251116_173045 --topics /camera/image_raw/compressed
```

### 재생하면서 확인

```bash
# Terminal 1: bag 재생
ros2 bag play data_20251116_173045

# Terminal 2: 이미지 확인 (rviz2)
rviz2

# Terminal 3: 관절 상태 확인
ros2 topic echo /lerobot/follower/joint_states
```

## 🔧 문제 해결

### 카메라가 보이지 않음 (신호 없음)

**원인**: 카메라 토픽이 발행되지 않음

**해결**:
```bash
# 토픽 확인
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep compressed

# 카메라 시스템 재시작
./stop_all.sh
./launch_all_background.sh
```

### GUI가 실행되지 않음

**원인**: PySide6 설치 안됨

**해결**:
```bash
conda activate lerobot_ros2
pip install PySide6
```

### 녹화 버튼을 눌렀지만 파일이 생성되지 않음

**원인**: ROS 2 환경이 제대로 source되지 않음

**해결**:
```bash
# 수동으로 환경 활성화 후 실행
source /opt/ros/jazzy/setup.bash
python3 ros2_bag_recorder_gui.py
```

### bag 파일이 너무 큼

**원인**: Compressed 이미지도 여전히 용량이 큼

**해결 방법**:
1. **해상도 낮추기** (USB 카메라)
   ```bash
   export USB_CAMERA_WIDTH=320
   export USB_CAMERA_HEIGHT=240
   ./launch_all_background.sh
   ```

2. **FPS 낮추기**
   ```bash
   export USB_CAMERA_FPS=15
   ./launch_all_background.sh
   ```

3. **PointCloud만 제외하고 녹화**
   - `ros2_bag_recorder_gui.py` 파일 수정
   - 110번째 줄에서 `/camera/depth/color/points` 제거

### GUI 종료 시 프로세스가 남아있음

**해결**:
```bash
# 수동으로 프로세스 종료
pkill -f "ros2 bag record"
```

## 📁 파일 구조

```
lerobot/
├── ros2_bag_recorder_gui.py      # GUI 애플리케이션
├── run_bag_recorder_gui.sh       # 실행 스크립트
├── BAG_RECORDER_GUI_GUIDE.md     # 이 가이드
└── data_YYYYMMDD_HHMMSS/         # 녹화된 bag 파일들
    ├── metadata.yaml
    └── data_YYYYMMDD_HHMMSS_0.db3
```

## ⚙️ 고급 설정

### 녹화할 토픽 변경

`ros2_bag_recorder_gui.py` 파일의 109-114번째 줄을 수정:

```python
# Topics to record
topics = [
    '/camera/color/image_raw/compressed',
    '/camera/depth/color/points',
    '/camera/image_raw/compressed',
    '/lerobot/follower/joint_states'
]
```

원하는 토픽을 추가하거나 제거하세요.

### 카메라 토픽 변경

`ros2_bag_recorder_gui.py` 파일의 57-70번째 줄을 수정:

```python
# Subscribe to camera topics
self.head_sub = self.create_subscription(
    CompressedImage,
    '/camera/camera/color/image_raw/compressed',  # <- 이 부분 수정
    self.head_image_callback,
    10
)

self.wrist_sub = self.create_subscription(
    CompressedImage,
    '/camera/image_raw/compressed',  # <- 이 부분 수정
    self.wrist_image_callback,
    10
)
```

### bag 파일 저장 위치 변경

`ros2_bag_recorder_gui.py` 파일의 293번째 줄을 수정:

```python
self.bag_process = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    cwd=os.path.expanduser('~/lerobot')  # <- 이 부분 수정
)
```

## 💡 사용 팁

### 1. 녹화 전 체크리스트
- [ ] 모든 카메라가 연결되어 있나요?
- [ ] `launch_all.sh` 또는 `launch_all_background.sh`가 실행 중인가요?
- [ ] GUI에서 Head View와 Wrist View가 모두 보이나요?
- [ ] 디스크 공간이 충분한가요? (최소 1GB 권장)

### 2. 녹화 중 주의사항
- 녹화 중에는 시스템을 종료하지 마세요
- 카메라 케이블을 빼지 마세요
- 네트워크 연결이 끊어지지 않도록 주의하세요

### 3. 키보드 단축키 활용
- **A, S, D 키**를 사용하면 마우스 없이 빠르게 녹화 제어 가능
- 한 손으로 로봇 조작, 다른 손으로 키보드 조작이 편리함
- 녹화 중에는 **S**(저장) 또는 **D**(취소)만 활성화됨

### 4. 파일 관리
- 주기적으로 오래된 bag 파일을 백업하거나 삭제하세요
- 중요한 데이터는 외부 저장소에 백업하세요

## 🎯 빠른 시작

```bash
# 1. 시스템 실행
./launch_all_background.sh

# 2. GUI 실행
./run_bag_recorder_gui.sh

# 3. GUI에서:
#    - "시작" 클릭
#    - 데이터 수집
#    - "저장" 클릭

# 4. 확인
cd ~/lerobot
ros2 bag info data_*
```

## 📚 관련 문서

- [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) - LeRobot 시스템 실행 가이드
- [ROS2_QUICK_START.md](ROS2_QUICK_START.md) - ROS2 빠른 시작 가이드
- [CLAUDE.md](CLAUDE.md) - LeRobot 전체 문서

---

**버전**: 1.0.0
**최종 업데이트**: 2025-11-16
