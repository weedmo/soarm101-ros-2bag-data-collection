# ROS2 Quick Start Guide

ROS2 기반 calibration과 teleoperation이 준비되었습니다!

## 설치 완료 사항

✅ Python 3.12 conda 환경 생성 (`lerobot_ros2`)
✅ ROS2 Jazzy 연동 완료
✅ LeRobot 설치 완료
✅ 편의 스크립트 생성 완료

## 사용 방법

### 1. Leader Calibration (Teleoperator)

```bash
./run_calibrate_ros2.sh \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyUSB1 \
    --teleop.id=leader
```

**다른 터미널에서 ROS2 서비스 호출:**

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# Calibration 시작
ros2 service call /lerobot_calibration_server/start_calibration std_srvs/srv/Trigger

# Calibration 상태 확인
ros2 service call /lerobot_calibration_server/get_status std_srvs/srv/Trigger

# Calibration 저장
ros2 service call /lerobot_calibration_server/save_calibration std_srvs/srv/Trigger

# 상태 모니터링 (선택사항)
ros2 topic echo /lerobot_calibration_server/calibration_status
```

### 2. Follower Calibration (Robot)

```bash
./run_calibrate_ros2.sh \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=follower
```

**다른 터미널에서 동일한 ROS2 서비스 사용**

### 3. Teleoperation 실행

Calibration이 완료되면 calibration 서버를 종료(Ctrl+C)하고 teleoperation을 시작하세요.

#### Terminal 1: Leader Node

```bash
./run_teleoperate_ros2.sh \
    --mode=leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyUSB1 \
    --teleop.id=leader \
    --topic=/lerobot/leader/joint_states \
    --rate=50
```

#### Terminal 2: Follower Node

```bash
./run_teleoperate_ros2.sh \
    --mode=follower \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=follower \
    --topic=/lerobot/leader/joint_states \
    --rate=50
```

#### Terminal 3: 모니터링 (선택사항)

```bash
source /opt/ros/jazzy/setup.bash

# Leader joint states 확인
ros2 topic echo /lerobot/leader/joint_states

# Follower joint states 확인
ros2 topic echo /lerobot/follower/joint_states

# Topic 주파수 확인
ros2 topic hz /lerobot/leader/joint_states

# 모든 topic 보기
ros2 topic list

# Topic 정보 보기
ros2 topic info /lerobot/leader/joint_states
```

## 직접 Conda 환경 사용하기

스크립트 대신 직접 환경을 활성화해서 사용할 수도 있습니다:

```bash
# Conda 환경 활성화
conda activate lerobot_ros2

# ROS2 환경 source
source /opt/ros/jazzy/setup.bash

# 이제 명령어 직접 사용 가능
lerobot-calibrate-ros2 --help
lerobot-teleoperate-ros2 --help
```

## 포트 찾기

디바이스 포트를 모르는 경우:

```bash
# 기존 lerobot 환경 사용
conda activate lerobot
lerobot-find-port

# 또는 직접 확인
ls -la /dev/ttyUSB*
ls -la /dev/ttyACM*
```

## Calibration 파일 위치

Calibration 데이터는 다음 위치에 저장됩니다:

```
~/.cache/lerobot/calibration/{device_id}.json
```

예:
- `~/.cache/lerobot/calibration/leader.json`
- `~/.cache/lerobot/calibration/follower.json`

## Troubleshooting

### 포트 권한 에러

```bash
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### Calibration 파일 삭제하고 다시 하기

```bash
rm ~/.cache/lerobot/calibration/leader.json
rm ~/.cache/lerobot/calibration/follower.json
```

### ROS2 노드 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
```

## 다음 단계

자세한 정보는 다음 문서를 참조하세요:

- [ROS2_INTEGRATION.md](ROS2_INTEGRATION.md) - 전체 ROS2 통합 가이드
- [CLAUDE.md](CLAUDE.md) - LeRobot 전체 문서
- [ROS2_USAGE.md](src/lerobot/robots/so101_follower/ROS2_USAGE.md) - SO101 ROS2 사용법
