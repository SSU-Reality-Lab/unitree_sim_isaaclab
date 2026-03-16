# RealityLab 설치 및 실행 가이드

RealityLab은 편의점 선반 시뮬레이션 환경(CS-Projects)을 Isaac Lab 위에서 구동하는 프로젝트입니다.
G1 로봇(Dex3 핸드)이 선반 위의 물건을 조작(쓰러진 물건 세우기, 정렬 등)하는 태스크를 수행합니다.

---

## 1. 사전 요구사항

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04+ |
| GPU | NVIDIA RTX (CUDA 12.1 이상) |
| Conda | Miniconda 또는 Anaconda |
| Git LFS | 필수 (USD 에셋 다운로드용) |

---

## 2. 프로젝트 클론

```bash
cd ~/Projects
git clone <unitree_sim_isaaclab_repo_url> unitree_sim_isaaclab
```

> `generated_scenes4/` 에셋 디렉토리는 별도로 받아서 `~/Projects/generated_scenes4/`에 배치합니다.

---

## 3. 자동 환경 설치

`auto_setup_env.sh`가 모든 의존성을 자동으로 설치합니다.

```bash
cd ~/Projects/unitree_sim_isaaclab
bash auto_setup_env.sh <isaac_version> <conda_env_name> [cuda_version]
```

**예시:**
```bash
# Isaac Sim 5.1 + CUDA 12.6
bash auto_setup_env.sh 5.1 realitylab cu126

# Isaac Sim 4.5 + CUDA 12.1 (기본값)
bash auto_setup_env.sh 4.5 realitylab
```

### 스크립트가 자동으로 하는 작업

| 단계 | 내용 |
|------|------|
| Phase 1 | IsaacLab, CycloneDDS, unitree_sdk2_python 레포 클론 |
| | git submodule 초기화, SSL 인증서 생성, USD 에셋 다운로드 |
| Phase 2 | CycloneDDS C 라이브러리 빌드 (`cyclonedds/install/`) |
| Phase 3 | Conda 환경 생성, PyTorch/Isaac Sim/Isaac Lab 설치 |
| | unitree_sdk2_python, teleimager, requirements.txt 설치 |

완료 후 디렉토리 구조:

```
~/Projects/
├── unitree_sim_isaaclab/   ← 메인 프로젝트
├── IsaacLab/               ← Isaac Lab 프레임워크
├── cyclonedds/             ← DDS 라이브러리 (빌드 완료)
├── unitree_sdk2_python/    ← Unitree Python SDK
└── generated_scenes4/      ← 씬 에셋 (별도 배치)
```

---

## 4. 환경변수 설정 (~/.bashrc)

`~/.bashrc`에 아래 내용을 추가합니다:

```bash
# ===== RealityLab 환경변수 =====

# CycloneDDS (SDK 빌드에 필수)
export CYCLONEDDS_HOME=~/Projects/cyclonedds/install

# CS-Projects 씬 에셋 경로 (generated_scenes4 디렉토리를 가리킴)
export CS_PROJECTS=~/Projects/generated_scenes4

# (선택) DDS 네트워크 인터페이스 설정 — 실제 로봇 연결 시 필요
# export CYCLONEDDS_URI=file:///home/$USER/Projects/custom/cyclonedds.xml

# ===== RealityLab 단축 명령어 =====

# Conda 환경 활성화
alias rl='conda activate realitylab'

# 프로젝트 디렉토리 이동
alias cdrl='cd ~/Projects/unitree_sim_isaaclab'

# CS-Projects 시뮬레이션 실행 (씬 번호를 인자로)
rl-run() {
    local idx="${1:-0}"
    cd ~/Projects/unitree_sim_isaaclab
    python sim_main.py \
        --device cpu \
        --enable_cameras \
        --task "CS-Projects-${idx}" \
        --enable_dex3_dds \
        --robot_type g129
}

# 등록된 씬 목록 확인
rl-list() {
    python -c "
import os
os.environ.setdefault('CS_PROJECTS', os.path.expanduser('~/Projects/generated_scenes4'))
from tasks.g1_tasks.cs_projects.cs_projects_scene_cfg import find_scene_jsons
scenes = find_scene_jsons()
for i, s in enumerate(scenes):
    print(f'  [{i:3d}] {os.path.relpath(s, os.environ[\"CS_PROJECTS\"])}')
print(f'\nTotal: {len(scenes)} scenes')
"
}
```

적용:

```bash
source ~/.bashrc
```

---

## 5. 실행 방법

### 5.1 기본 실행

```bash
rl          # conda 환경 활성화
cdrl        # 프로젝트 디렉토리 이동
rl-run 0    # 씬 #0 실행
```

또는 직접:

```bash
conda activate realitylab
cd ~/Projects/unitree_sim_isaaclab

python sim_main.py \
    --device cpu \
    --enable_cameras \
    --task CS-Projects-0 \
    --enable_dex3_dds \
    --robot_type g129
```

### 5.2 주요 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--task CS-Projects-{N}` | 로드할 씬 번호 (0부터 시작) | — |
| `--device cpu` | 시뮬레이션 디바이스 | — |
| `--enable_cameras` | 카메라 활성화 | off |
| `--enable_dex3_dds` | Dex3 핸드 DDS 활성화 | off |
| `--robot_type g129` | 로봇 타입 (g129 / h1_2) | g129 |
| `--replay_data` | 데이터 리플레이 모드 | off |
| `--file_path <dir>` | 리플레이 데이터 경로 | — |
| `--physics_dt 0.005` | 물리 타임스텝 (초) | 0.005 |
| `--no_render` | 렌더링 비활성화 (헤드리스) | off |

### 5.3 데이터 수집/리플레이

```bash
# 리플레이
python sim_main.py --device cpu --enable_cameras \
    --task CS-Projects-0 --enable_dex3_dds --robot_type g129 \
    --replay_data --file_path /path/to/recorded_data

# 데이터 생성 (augmentation)
python sim_main.py --device cpu --enable_cameras \
    --task CS-Projects-0 --enable_dex3_dds --robot_type g129 \
    --replay_data --file_path /path/to/data \
    --generate_data --generate_data_dir ./output
```

---

## 6. 씬 로딩 구조

### 6.1 전체 흐름

```
CS_PROJECTS 환경변수
       │
       ▼
 __init__.py ── find_scene_jsons() ── 모든 scene_*.json 탐색
       │
       ▼
 SCENE_REGISTRY = {0: "task1/.../scene_0000.json", 1: ..., ...}
       │
       ▼
 gym.register("CS-Projects-0", ...) ~ gym.register("CS-Projects-230", ...)
       │
       ▼  (--task CS-Projects-N 지정 시)
       │
 get_env_cfg_class(N) ── SCENE_REGISTRY[N] 조회
       │
       ▼
 CSProjectsEnvCfg.__post_init__()
       │
       ▼
 inject_scene_assets(scene_json_path)
       │
       ▼
 build_scene_assets() ── JSON 파싱 + 에셋 빌드
       │
       ├── 선반: KinematicUsdFileCfg (SDF 충돌체)
       ├── 아이템: RigidUsdFileCfg (동적 강체)
       └── 배치 좌표: plane JSON에서 계산
```

### 6.2 씬 JSON → Isaac Lab 변환

`build_scene_assets()`가 씬 JSON을 읽고 Isaac Lab 에셋으로 변환합니다:

1. **선반** (`shelf_real.usd`) → `AssetBaseCfg` + `KinematicUsdFileCfg`
   - Kinematic rigid body (움직이지 않음)
   - SDF 메시 충돌체 적용 (아이템이 관통하지 않도록)

2. **아이템** (도형 USD 파일들) → `RigidObjectCfg` + `RigidUsdFileCfg`
   - 동적 강체 (중력, 충돌, 파지 가능)
   - Convex decomposition 충돌체
   - 질량: 부피 × 300 kg/m³ (0.05~2.0 kg 클램프)

3. **배치 좌표 계산**:
   - `planes_fixed_real/plane_0X.json` → 선반 각 층의 좌표계 (centroid, u/v/n 벡터)
   - `zone_config` → 각 층에 어떤 아이템을 몇 개, 어떤 간격으로 배치할지
   - 선반 로컬 → 월드 변환: 90° X축 회전 적용

### 6.3 씬 에셋 디렉토리 구조

```
generated_scenes4/
├── Prop/shelf/
│   ├── shelf_real.usd              ← 선반 3D 모델
│   └── planes_fixed_real/          ← 선반 각 층의 좌표 데이터
│       ├── plane_00.json           ← 1층 (최하단)
│       ├── plane_01.json           ← 2층
│       ├── plane_02.json           ← 3층
│       └── plane_03.json           ← 4층 (최상단)
├── asset/shapes/                   ← 12종 도형 프리미티브
│   ├── bottle1/bottle1.usd
│   ├── can1/can1.usd
│   ├── cube5x5/cube5x5.usd
│   └── ...
├── task1_LLM_scene/                ← Task 1: 쓰러진 물건 세우기 (1x1)
│   ├── train/normal/               ← 학습용 (119 scenes)
│   └── test/normal/
├── task2_LLM_scene/                ← Task 2: 비정렬 물건 찾아 고치기 (3x4)
│   ├── train/normal/               ← 학습용 (3 scenes)
│   └── test/normal/
├── task3_LLM_scene/                ← Task 3: 미구현 (트롤리 → 1x1)
├── task4_LLM_scene/                ← Task 4: 미구현 (트롤리 → 4x3)
└── misc_LLM_scene/                 ← 일반 배치 (태스크 목표 없음, 109 scenes)
```

### 6.4 씬 인덱스 매핑

`find_scene_jsons()`가 `CS_PROJECTS` 하위의 모든 `scene_*.json`을 재귀 탐색하고 정렬합니다.
총 **231개** 씬이 `CS-Projects-0` ~ `CS-Projects-230`으로 등록됩니다.

씬 번호는 파일 경로의 알파벳 순서로 결정됩니다:
- `CS-Projects-0` ~ 끝까지: misc → task1 → task2 순서 (디렉토리명 정렬)

`rl-list` 명령어로 전체 매핑을 확인할 수 있습니다.

---

## 7. 태스크 정의

| Task | 선반 | 목표 | 이상 항목 | 상태 |
|------|------|------|-----------|------|
| 1 | 1x1 | 쓰러진 물건 세우기 | 1개 | Active |
| 2 | 3x4 | 비정렬 물건 찾아 고치기 | 1~2개 | Active |
| 3 | 1x1 | 트롤리 → 선반 배치 | 0 | Pending |
| 4 | 4x3 | 트롤리 → 빈 슬롯 채우기 | 1~2 빈칸 | Pending |

각 씬 JSON에는 `anomalies` 배열이 포함되어 있어 정답 데이터(목표 위치/회전)를 제공합니다.

---

## 8. 로봇 구성

- **로봇**: G1 29-DoF + Dex3 양손 (14관절)
- **스폰 위치**: (-0.15, -0.403, 0.76)
- **카메라 3대**: 정면(head), 좌측 손목(left_wrist), 우측 손목(right_wrist)
- **DDS 도메인 ID**: 1 (시뮬레이션), 0 (실제 로봇)

---

## 9. 트러블슈팅

### CycloneDDS 빌드 오류
```
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME
```
→ `~/.bashrc`에 `export CYCLONEDDS_HOME=~/Projects/cyclonedds/install` 확인

### CS-Projects 씬 미등록
```
[CS-Projects] CS_PROJECTS=... is not a valid directory
```
→ `export CS_PROJECTS=~/Projects/generated_scenes4` 경로가 정확한지 확인

### 아이템이 선반을 관통
→ 선반이 `KinematicUsdFileCfg`로 스폰되는지 확인 (SDF 충돌체 필요)

### Isaac Sim 5.0/5.1 libstdc++ 오류
```bash
conda install -y -c conda-forge libstdcxx-ng
```
