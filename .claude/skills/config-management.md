---
name: config-management
description: >
  설정 파일 관리 패턴 — YAML 스키마 설계, 환경 변수, .env 시크릿,
  pydantic-settings 검증, 12-factor 원칙, 환경별 설정 분리.
  config/ 파일 작성·수정, 환경 변수 도입, 시크릿 관리 시 활성화.
when_to_use: >
  config/*.yaml 추가/수정, 환경 변수 도입, .env 관리,
  설정 검증 로직 작성, 환경별 (개발/스테이징/프로덕션) 설정 분리 시.
---

# 설정 관리 (Configuration Management)

> 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-4. 좌표·임계값·시간 상수는 코드 하드코딩 금지.

## 1. 분류 — 무엇을 어디에 둘 것인가

| 분류 | 저장 위치 | 예시 | 보안 |
|------|-----------|------|------|
| **불변 도메인 데이터** | `config/*.yaml` (git) | 슬롯 좌표, 공구 기하 | 공개 가능 |
| **운영 파라미터** | `config/*.yaml` (git) | FOD 임계 시간, 속도 한계 | 공개 가능 |
| **환경별 값** | 환경 변수 (`.env`) | DB host, ROS_DOMAIN_ID | 공개 가능 |
| **시크릿** | `.env` (gitignored) | API 키, 토큰, 비밀번호 | 절대 git 금지 |
| **개인 설정** | `.claude/settings.local.json` (gitignored) | 디버그 플래그 | 공개 가능 |

## 2. 이 프로젝트 config 파일 구조

```
config/
├── staging_area.yaml    # 공구별 Staging Area 좌표
├── toolbox.yaml         # 슬롯 좌표 + 공구 기하 + 공구 클래스
├── hand_eye.yaml        # 카메라-엔드이펙터 변환 (gitignored, 장비별 로컬)
├── robot_poses.yaml     # home, scan 포즈
├── fod.yaml             # FOD 임계 시간 등 운영 파라미터
└── runtime.yaml         # 비-시크릿 런타임 상수 (robot_model, whisper_model_size, operator_id)

.env                     # 시크릿 (gitignored)
.env.example             # 변수 이름만 (git 포함)
```

## 3. YAML 스키마 설계 원칙

### 명시적 단위
```yaml
# ✅ 단위가 키 이름에 포함
fod:
  checkout_timeout_minutes: 10
  alert_delay_seconds: 30

# ❌ 단위 불명확
fod:
  checkout_timeout: 10        # 분? 초?
```

### 명시적 frame
```yaml
# config/staging_area.yaml
# frame: robot_base_link (모든 좌표)
slots:
  screwdriver_phillips_small:
    position_m: [0.55, 0.10, 0.08]
    quaternion: [0.0, 0.0, 0.0, 1.0]   # [x, y, z, w]
  wrench_8mm:
    position_m: [0.55, 0.20, 0.08]
    quaternion: [0.0, 0.0, 0.0, 1.0]
```

### 일관된 명명 (snake_case)
```yaml
# ✅
robot_poses:
  home: [0.0, 0.5, -1.2, 0.0, 0.7, 0.0]
  scan_left: [0.5, 0.3, -1.0, 0.0, 0.7, 0.0]

# ❌
RobotPoses:
  Home: ...
  scanLeft: ...
```

### enum 명시
```yaml
# ✅ 허용 값 주석으로 명시
fod:
  default_action: missing    # missing | warn | ignore
```

### 버전 필드 (마이그레이션 대비)
```yaml
# config/staging_area.yaml
schema_version: 1
# v1: 단일 슬롯 per 공구
# v2 예정: 다중 슬롯 + 우선순위

slots:
  ...
```

## 4. 환경 변수 (`.env` + `.env.example`)

### `.env.example` (git 포함, 값 없음)
```bash
# ROS2
ROS_DOMAIN_ID=42

# DB
DB_HOST=localhost
DB_PORT=5432
DB_USER=robot_arm
DB_PASSWORD=          # 빈 값 — .env에 실제 값

# Cloud / VLA
HUGGINGFACE_TOKEN=
LAMBDA_LABS_API_KEY=
VLA_CHECKPOINT_URL=

# Doosan
DOOSAN_HOST=192.168.137.100
DOOSAN_PORT=12345
```

### `.env` (gitignored, 실제 값)
```bash
ROS_DOMAIN_ID=42
DB_HOST=localhost
DB_PORT=5432
DB_USER=robot_arm
DB_PASSWORD=actual_secret_here

HUGGINGFACE_TOKEN=hf_xxxxx
LAMBDA_LABS_API_KEY=ll-xxxxx
VLA_CHECKPOINT_URL=s3://my-bucket/openvla-finetuned.ckpt

DOOSAN_HOST=192.168.137.100
DOOSAN_PORT=12345
```

### 로드 방법
```python
# ✅ python-dotenv
from dotenv import load_dotenv
import os

load_dotenv()  # .env 읽어서 os.environ에 주입

doosan_host = os.environ["DOOSAN_HOST"]              # 필수 — 없으면 KeyError
db_password = os.environ.get("DB_PASSWORD", "")     # 선택 — 기본값

# ❌ 절대 금지
DOOSAN_HOST = "192.168.137.100"   # 하드코딩
DB_PASSWORD = "actual_secret"      # 시크릿 하드코딩
```

## 5. pydantic-settings (권장) — 검증 + 타입 안전

```python
# settings.py
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import SecretStr, Field

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_prefix="")

    # 필수 (env에 없으면 시작 실패)
    doosan_host: str
    doosan_port: int = Field(ge=1, le=65535)

    # 선택
    ros_domain_id: int = 42

    # 시크릿 (로그에 출력 안 됨)
    db_password: SecretStr
    huggingface_token: SecretStr | None = None

settings = Settings()

# 사용
arm = DooSanArm(host=settings.doosan_host, port=settings.doosan_port)
db.connect(password=settings.db_password.get_secret_value())
```

### 장점
- 시작 시 자동 검증 (잘못된 타입 → 즉시 실패)
- IDE 자동완성
- `SecretStr`은 print/log 시 `**********` 표시

## 6. YAML 로딩 + 검증

```python
import yaml
from pydantic import BaseModel, Field
from pathlib import Path

class StagingSlot(BaseModel):
    position_m: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]

    @property
    def x(self) -> float: return self.position_m[0]

class StagingAreaConfig(BaseModel):
    schema_version: int = Field(ge=1)
    slots: dict[str, StagingSlot]

def load_staging_config(path: Path = Path("config/staging_area.yaml")) -> StagingAreaConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    return StagingAreaConfig(**raw)   # 자동 검증

# 사용
config = load_staging_config()
phillips_slot = config.slots["screwdriver_phillips_small"]
print(phillips_slot.x)
```

> `yaml.safe_load` 사용. `yaml.load`는 임의 코드 실행 가능 — 절대 금지.

## 7. 환경별 설정 (개발/스테이징/프로덕션)

### 패턴 A: 환경 변수로 선택
```bash
# 실행 시
ENVIRONMENT=development ./run.sh --track A
ENVIRONMENT=production ./run.sh --track A
```
```python
env = os.environ.get("ENVIRONMENT", "development")
config_path = Path(f"config/{env}/staging_area.yaml")
```

### 패턴 B: 기본 + 오버라이드
```yaml
# config/base.yaml — 공통
fod:
  checkout_timeout_minutes: 10

# config/dev.yaml — 개발용 오버라이드
fod:
  checkout_timeout_minutes: 1   # 빠른 테스트
```

```python
base = yaml.safe_load(Path("config/base.yaml").read_text())
override = yaml.safe_load(Path(f"config/{env}.yaml").read_text())
merged = deep_merge(base, override)
```

## 8. ROS2 파라미터 — 노드별 설정

ROS2 노드는 자체 파라미터 시스템 사용. YAML과 통합 가능.

```yaml
# config/voice_node.yaml
voice_node:
  ros__parameters:
    whisper_model: small
    sample_rate: 16000
    vad_threshold: 0.5
```

```bash
# 런칭 시 적용
ros2 launch voice voice.launch.py params:=config/voice_node.yaml
```

```python
# 노드 내부
class WhisperNode(Node):
    def __init__(self):
        super().__init__("voice_node")
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("sample_rate", 16000)
        model = self.get_parameter("whisper_model").value
```

## 9. 런타임 변경 가능 vs 불가능

| 종류 | 변경 가능 시점 | 예시 |
|------|---------------|------|
| **컴파일 타임** | 빌드 시 | 패키지 의존성 |
| **시작 타임** | 노드 실행 시 (YAML) | 슬롯 좌표, 모델 경로 |
| **런타임** | 실행 중 (서비스) | 디버그 레벨, FOD 임계값 |

런타임 변경 필요한 파라미터는 ROS2 dynamic_reconfigure 또는 서비스 인터페이스 제공:
```python
self.create_service(SetFODTimeout, '/fod/set_timeout', self.on_set_timeout)
```

## 10. 시크릿 관리 — 노출 방지

### Pre-commit hook (필수)
```yaml
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/Yelp/detect-secrets
    rev: v1.5.0
    hooks:
      - id: detect-secrets
        args: ['--baseline', '.secrets.baseline']
```

```bash
pip install pre-commit detect-secrets
pre-commit install
# 이후 git commit 때마다 자동 검사
```

### 이미 commit한 시크릿 — 즉시 처리
1. **키 revoke** (가장 우선)
2. `git filter-repo` 또는 BFG로 history에서 제거
3. force-push (팀 사전 알림)
4. 모든 팀원이 다시 clone 또는 rebase

자세히는 [`skills/git-conventions.md`](git-conventions.md) §8 참조.

## 11. 검증 — CI에서 자동화

```yaml
# .github/workflows/config-check.yml
name: config validation
on: [push, pull_request]
jobs:
  validate:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
      - run: pip install pyyaml pydantic
      - name: Validate all config YAMLs
        run: |
          python -c "
          from settings import load_all_configs
          load_all_configs()  # 검증 실패 시 비zero exit
          "
      - name: Check for accidental secrets
        uses: trufflesecurity/trufflehog@main
        with:
          base: main
          head: HEAD
```

## 12. 흔한 함정

### ❌ 환경 변수에 큰 데이터
- ✅ 환경 변수: 짧은 값 (host, key, flag)
- ✅ YAML: 구조화된 큰 데이터 (좌표 행렬, 공구 목록)

### ❌ 검증 없는 YAML 로드
```python
config = yaml.safe_load(open("config.yaml"))
slot = config["slots"]["screwdriver_phillips_small"]  # KeyError 가능
```
✅ pydantic 검증 후 사용

### ❌ 시크릿을 logging
```python
logger.info(f"Connecting with token={token}")   # 로그에 시크릿 노출
```
✅ `SecretStr` 사용하면 자동 마스킹

### ❌ 환경별 config 누락
- 개발에서 `fod.timeout=1분`으로 테스트했는데 프로덕션 default가 사용됨
- ✅ 환경별 명시적 분리 + CI에서 모든 환경 검증

### ❌ Default를 안전하지 않게
```python
# 위험 — 환경 변수 없으면 production에 dev 키 사용
api_key = os.environ.get("API_KEY", "dev-key-12345")
```
✅ 필수 시크릿은 default 없이
```python
api_key = os.environ["API_KEY"]   # KeyError로 명시적 실패
```

## 13. 참고

- 12-Factor App: <https://12factor.net/ko/config>
- pydantic-settings: <https://docs.pydantic.dev/latest/concepts/pydantic_settings/>
- python-dotenv: <https://pypi.org/project/python-dotenv/>
- detect-secrets: <https://github.com/Yelp/detect-secrets>
- 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-4, [`.claude/rules/process.md`](../rules/process.md) P-3
