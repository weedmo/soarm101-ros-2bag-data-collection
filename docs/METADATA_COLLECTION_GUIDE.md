# ROS 2 Bag Recorder with Metadata Collection 가이드

VLA(Vision-Language-Action) 및 RFM(Robotics Foundation Model) 학습을 위한 메타데이터 수집 기능이 포함된 GUI입니다.

## 📋 메타데이터의 중요성

메타데이터는 단순한 파일 정보가 아닙니다. **모델 학습에 직접 사용되는 핵심 입력값**입니다.

### VLA/RFM 관점에서의 메타데이터 활용

- **Instruction (지시어)**: VLA 모델의 언어 입력(Prompt)
  - 예: "빨간색 블록을 파란색 그릇으로 옮겨"
- **Task Name (작업명)**: 데이터 필터링 및 성능 평가 기준
  - 예: "grasp", "push", "place"
- **Context (환경 정보)**: 편향성 분석 및 일반화 성능 향상
  - operator, camera_name, robot_model 등

## 🚀 실행 방법

### 1. 시스템 준비

```bash
# 모든 컴포넌트 실행
./launch_all_background.sh
```

### 2. GUI 실행

```bash
./run_bag_recorder_with_metadata.sh
```

## 🎮 GUI 구성

### 좌측 패널: 카메라 뷰 & 녹화 컨트롤
- **Head View**: RealSense 외부 카메라
- **Wrist View**: RealSense 손목 카메라
- **녹화 버튼**: 시작/저장/취소 (A/S/D 단축키)

### 우측 패널: 메타데이터 입력

#### 1. Task Information (작업 정보)
- **Task ID**: 고유한 작업 식별자
  - 예: `pick_and_place_red_cube_001`
- **Task Name**: 사람이 읽을 수 있는 작업명
  - 예: `Pick and Place (Red Cube to Blue Bowl)`
- **Task Type**: 작업 유형 (드롭다운)
  - `pick_and_place`, `grasp`, `push`, `pull`, `pour`, `stack`, etc.
- **Instruction**: 자연어 지시어 (여러 줄)
  - 예: `"Place the red cube into the blue bowl."`
- **Tags**: 태그 (쉼표로 구분)
  - 예: `manipulation, object_transfer, red_cube`

#### 2. Collection Context (수집 컨텍스트)
- **Operator**: 작업자 이름
  - 예: `junmo`
- **Robot Model**: 로봇 모델명 (드롭다운)
  - `RB-Y1`, `SO-ARM100`, `SO101`, `UR5e`, `Franka Emika`, etc.
- **Location**: 작업 위치
  - 예: `alchera_lab_01`
- **Environment Notes**: 환경 특이사항
  - 예: `"Sunny day, strong natural light from window."`

#### 3. Hardware Configuration (하드웨어 구성)
- **Head Camera Name**: 외부 카메라 이름
  - 기본값: `realsense_d435i_external`
- **Wrist Camera Name**: 손목 카메라 이름
  - 기본값: `realsense_d435i_wrist`
- **Gripper Model**: 그리퍼 모델 (드롭다운)
  - `rg2`, `robotiq_2f_85`, `schunk_gripper`, etc.

#### 4. Recording Topics (녹화 토픽 선택)
- **🔄 Refresh Topics**: 현재 발행 중인 토픽 목록을 새로고침
- **체크박스 리스트**: 녹화할 토픽 선택
  - 기본적으로 다음 4개 토픽이 선택됨:
    - `/camera/color/image_raw/compressed`
    - `/camera/depth/color/points`
    - `/camera/image_raw/compressed`
    - `/lerobot/follower/joint_states`
  - 필요에 따라 추가 토픽 선택 가능
  - 토픽 타입이 자동으로 메타데이터에 포함됨

#### 5. Custom Fields (사용자 정의 필드)
- **Task Success**: 작업 성공 여부 (true/false)
- **Failure Reason**: 실패 시 이유

## 📝 메타데이터 스키마 (Schema Version 1.0.0)

자동 생성되는 JSON 파일 구조:

```json
{
  "schema_version": "1.0.0",
  "collection_uuid": "abc-123-xyz-789",
  "task_info": {
    "task_id": "pick_and_place_red_cube_001",
    "task_name": "Pick and Place (Red Cube to Blue Bowl)",
    "instruction": "Place the red cube into the blue bowl.",
    "task_type": "pick_and_place",
    "tags": ["manipulation", "object_transfer", "red_cube"]
  },
  "collection_context": {
    "operator": "junmo",
    "robot_model": "SO101",
    "location": "alchera_lab_01",
    "environment_notes": "Sunny day, strong natural light."
  },
  "timestamps": {
    "start_utc": "2025-11-16T08:58:00Z",
    "end_utc": "2025-11-16T08:58:45Z",
    "duration_sec": 45.0
  },
  "data_provenance": {
    "rosbag_filename": "20251116_175800_pick_place_red_cube",
    "rosbag_size_mb": 1024.5
  },
  "hardware_config": {
    "cameras": [
      {
        "cam_name": "realsense_d435i_external",
        "type": "depth_camera",
        "position": "external"
      },
      {
        "cam_name": "realsense_d435i_wrist",
        "type": "depth_camera",
        "position": "wrist"
      }
    ],
    "gripper": {
      "model": "rg2",
      "type": "2_finger"
    }
  },
  "recorded_topics": [
    {
      "name": "/camera/color/image_raw/compressed",
      "message_type": "sensor_msgs/msg/CompressedImage"
    },
    {
      "name": "/camera/depth/color/points",
      "message_type": "sensor_msgs/msg/PointCloud2"
    },
    {
      "name": "/camera/image_raw/compressed",
      "message_type": "sensor_msgs/msg/CompressedImage"
    },
    {
      "name": "/lerobot/follower/joint_states",
      "message_type": "sensor_msgs/msg/JointState"
    }
  ],
  "custom_fields": {
    "is_success": true,
    "failure_reason": null
  }
}
```

## 📁 파일 명명 규칙

**형식**: `YYYYMMDD_HHMMSS_TaskName`

예시:
- Rosbag: `20251116_175800_pick_place_red_cube/`
- Metadata: `20251116_175800_pick_place_red_cube.json`

**1:1 매칭**: 동일한 base name을 사용하여 데이터와 메타데이터를 연결합니다.

## 🔄 워크플로우

### 1. 녹화 준비
1. GUI 실행
2. 우측 패널에서 메타데이터 입력
   - Task ID, Task Name, Instruction, Tags 등
   - Operator, Robot Model, Location 등
3. **Recording Topics 섹션에서 녹화할 토픽 선택**
   - 🔄 Refresh Topics 버튼으로 현재 토픽 목록 갱신
   - 필요한 토픽 체크/언체크
   - 최소 1개 이상의 토픽 선택 필요
4. 카메라 뷰에서 영상 확인

### 2. 녹화 시작 (A 키 또는 시작 버튼)
- Collection UUID 자동 생성
- 시작 timestamp 기록
- 선택된 토픽으로 Rosbag 녹화 시작
- 토픽 개수와 이름이 로그에 표시됨

### 3. 데이터 수집
- 로봇 조작 수행
- 타이머로 녹화 시간 확인

### 4. 녹화 종료 (S 키 또는 저장 버튼)
- Rosbag 프로세스 종료 (SIGINT)
- 종료 timestamp 기록
- **선택된 토픽의 타입 정보 자동 수집**
- **메타데이터 JSON 파일 자동 생성**
- Rosbag과 동일한 base name으로 저장

## 📊 생성되는 파일

녹화 완료 후 다음 파일들이 생성됩니다:

```
~/lerobot/
├── data/
│   └── 20251116_175800_pick_place_red_cube/
│       ├── metadata.yaml
│       └── 20251116_175800_pick_place_red_cube_0.db3
└── 20251116_175800_pick_place_red_cube.json  ← 메타데이터
```

## 🔍 메타데이터 확인

```bash
cd ~/lerobot

# JSON 파일 확인
cat 20251116_175800_pick_place_red_cube.json | jq

# 특정 필드 추출
jq '.task_info.instruction' 20251116_175800_pick_place_red_cube.json

# Collection UUID 확인
jq '.collection_uuid' 20251116_175800_pick_place_red_cube.json
```

## 💡 모범 사례 (Best Practices)

### Instruction 작성 가이드

**좋은 예시:**
```
"Pick up the red cube from the table and place it into the blue bowl."
"Grasp the bottle with your gripper and pour water into the cup."
"Open the top drawer and retrieve the green object."
```

**나쁜 예시:**
```
"Do it"  # 너무 모호함
"Move"   # 구체적이지 않음
""       # 비어있음
```

### Tags 작성 가이드

태그는 데이터셋 필터링과 검색에 사용됩니다. 쉼표로 구분하여 입력하세요.

**추천 카테고리:**
- **작업 유형**: `manipulation`, `navigation`, `grasping`, `assembly`
- **객체**: `red_cube`, `blue_bowl`, `bottle`, `drawer`
- **속성**: `fragile`, `heavy`, `transparent`, `deformable`
- **난이도**: `easy`, `medium`, `hard`
- **환경**: `cluttered`, `dynamic`, `static`

**좋은 예시:**
```
manipulation, object_transfer, red_cube
grasping, fragile, glass_object, careful
navigation, obstacle_avoidance, dynamic_environment
```

**나쁜 예시:**
```
test, test2, test3  # 의미 없는 태그
very_long_and_detailed_tag_that_describes_everything  # 너무 구체적
```

### Task ID 명명 규칙

**형식**: `{task_type}_{object}_{sequence_number}`

**예시:**
- `pick_and_place_red_cube_001`
- `grasp_bottle_002`
- `push_block_003`

### 환경 메모 작성

다음 정보를 포함하면 좋습니다:
- 조명 조건 (자연광, 인공 조명)
- 테이블 색상 및 재질
- 배경 복잡도
- 특이사항 (그림자, 반사 등)

## 🎯 데이터셋 구축 워크플로우

### 1. 시나리오 계획
```
- Task: Pick and Place (Red Cube to Blue Bowl)
- Variations:
  - 10회: 정상 조명
  - 10회: 강한 조명
  - 10회: 어두운 조명
```

### 2. 메타데이터 템플릿 준비
- Task ID 시퀀스 정의
- Instruction 표준화

### 3. 반복 수집
```bash
# 각 시도마다:
1. GUI 실행
2. Task ID 업데이트 (예: _001 → _002)
3. Environment Notes 업데이트
4. 녹화 시작 (A)
5. 작업 수행
6. 녹화 저장 (S)
7. Success 여부 체크
```

### 4. 품질 검증
```bash
# 모든 JSON 파일의 Instruction 확인
for f in *.json; do
  echo "=== $f ==="
  jq '.task_info.instruction' $f
done

# 성공률 계산
total=$(ls *.json | wc -l)
success=$(jq -r '.custom_fields.is_success' *.json | grep true | wc -l)
echo "Success rate: $success / $total"

# 특정 태그를 가진 데이터셋 찾기
for f in *.json; do
  if jq -e '.task_info.tags | index("manipulation")' $f > /dev/null; then
    echo "Found manipulation task: $f"
  fi
done

# 태그별 데이터셋 개수
echo "Tag distribution:"
jq -r '.task_info.tags[]' *.json | sort | uniq -c
```

## 🔧 고급 기능

### 메타데이터 일괄 업데이트

Python 스크립트로 여러 JSON 파일을 일괄 수정:

```python
import json
from pathlib import Path

# 모든 JSON 파일 찾기
json_files = Path('.').glob('*.json')

for json_file in json_files:
    with open(json_file, 'r') as f:
        metadata = json.load(f)

    # Location 일괄 업데이트
    metadata['collection_context']['location'] = 'alchera_lab_01'

    # 태그 일괄 추가 (기존 태그 유지)
    if 'tags' not in metadata['task_info']:
        metadata['task_info']['tags'] = []

    # 모든 데이터에 'dataset_v1' 태그 추가
    if 'dataset_v1' not in metadata['task_info']['tags']:
        metadata['task_info']['tags'].append('dataset_v1')

    with open(json_file, 'w') as f:
        json.dump(metadata, f, indent=4, ensure_ascii=False)
```

### 스키마 버전 관리

향후 스키마가 변경될 경우:

```python
def migrate_v1_to_v2(metadata):
    """Migrate metadata from v1.0.0 to v2.0.0"""
    if metadata['schema_version'] == '1.0.0':
        # Add new fields
        metadata['schema_version'] = '2.0.0'
        metadata['new_field'] = 'default_value'
    return metadata
```

## 📚 관련 문서

- [BAG_RECORDER_GUI_GUIDE.md](BAG_RECORDER_GUI_GUIDE.md) - 기본 녹화 GUI 가이드
- [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) - LeRobot 시스템 실행 가이드
- [ROS2_QUICK_START.md](ROS2_QUICK_START.md) - ROS2 빠른 시작 가이드

## 🎯 빠른 시작

```bash
# 1. 시스템 실행
./launch_all_background.sh

# 2. 메타데이터 수집 GUI 실행
./run_bag_recorder_with_metadata.sh

# 3. GUI에서:
#    - Task Info 입력 (Task ID, Name, Type, Instruction)
#    - Tags 입력 (예: manipulation, red_cube, object_transfer)
#    - Operator 입력
#    - Robot Model 선택 (예: SO101)
#    - Recording Topics에서 녹화할 토픽 선택 (기본 4개 체크됨)
#    - 'A' 키로 녹화 시작
#    - 작업 수행
#    - 'S' 키로 저장

# 4. 결과 확인
cd ~/lerobot
ls -la *.json

# 5. 메타데이터 내용 확인
cat 20251116_175800_pick_place_red_cube.json | jq

# 6. 녹화된 토픽 확인
jq '.recorded_topics' 20251116_175800_pick_place_red_cube.json
```

---

**버전**: 1.0.0 (Schema Version 1.0.0)
**최종 업데이트**: 2025-11-16
