# 🚀 Astrobee SE2 Trajectory Optimization
이 프로젝트는 2D 환경에서 Astrobee 로봇이 장애물을 피해 목표 지점까지 이동하는 **최적 경로(Trajectory)** 를 계산하는 시뮬레이션입니다.

SCP (Sequential Convex Programming) 알고리즘을 사용하며, MATLAB으로 시나리오를 설정하고 결과를 시각화하며, Julia가 백엔드에서 최적화 연산을 수행하는 구조로 되어 있습니다.

## 📁 파일 구조 (File Structure)
Plaintext
```bash
Astrobee_Project/
├── main.jl                # [실행] Julia 최적화 메인 스크립트
├── src/                   # Julia 소스 코드 (수정 불필요)
│   ├── parameters.jl      # 로봇/환경 파라미터 로딩
│   ├── definition.jl      # SCP 문제 정의 (동역학, 제약조건)
│   ├── solver.jl          # GuSTO 알고리즘 솔버 설정
│   └── user_config.jl     # MATLAB에서 생성한 Config CSV 로더
├── Mission_Control.m      # [설정] MATLAB 시나리오 설정 및 Config 생성기
├── Plot_Astrobee2D.m      # [시각화] 결과 그래프 및 애니메이션
├── trajectory.m           # [유틸] 궤적 데이터 처리용 MATLAB 함수
├── Project.toml           # 패키지 의존성 파일
└── Manifest.toml          # 패키지 버전 잠금 파일 (Julia 1.7.1 전용)
```
---

## ✔️ Step 1 — Julia 설치하기

가장 권장되는 방법은 **Juliaup (Julia 버전 관리자)** 사용입니다.

### Windows 사용자

```bash
winget install julia -s msstore
```

### Mac / Linux 사용자

```bash
curl -fsSL https://install.julialang.org | sh
```

## ✔️ Step 2 — Julia 1.7.1 설치 및 기본 버전 설정

```bash
juliaup add 1.7.1
juliaup default 1.7.1
julia --version
```
[MATLAB]
설정(Mission_Control.m) 및 시각화(Plot_Astrobee2D.m)를 위해 필요합니다.

---
## ⚙️ 2. 초기 설정 (Installation)
최초 1회, Julia 패키지들을 설치하고 환경을 구축해야 합니다.

터미널 열기: 프로젝트 폴더(Astrobee_Project)로 이동합니다.

패키지 설치: 아래 명령어를 입력하여 필요한 라이브러리를 다운로드합니다.

```Bash

julia --project=. -e 'using Pkg; Pkg.instantiate()'
```
네트워크 상황에 따라 시간이 소요될 수 있습니다.
---
## 🚀 3. 실행 방법 (Workflow)
이 프로젝트는 MATLAB(설정) → Julia(계산) → MATLAB(결과) 순서로 진행됩니다.

[Step 1] 시나리오 설정 (MATLAB)
MATLAB에서 Mission_Control.m 파일을 엽니다.

코드 상단의 사용자 설정 구역을 수정하여 시뮬레이션 환경을 정의합니다.
```Bash
N: 타임 스텝 수

tf: 총 비행 시간

obstacles: 장애물 위치 및 크기 [x, y, rx, ry] (타원 지원)

start_state / goal_state: 시작 및 목표 상태
```
스크립트를 실행(F5)합니다.

맵 창이 뜨면 마우스로 안전한 경유지(Waypoint)를 클릭합니다. (완료 시 Enter)

실행이 끝나면 폴더에 config_*.csv 파일들이 생성됩니다.

  
[Step 2] 최적화 수행 (Julia)
터미널에서 아래 명령어를 실행하여 최적 경로를 계산합니다.

```Bash

julia main.jl
```
계산이 성공하면 astrobee_result.csv 파일이 생성됩니다.

  
[Step 3] 결과 시각화 (MATLAB)
MATLAB에서 Plot_Astrobee2D.m 파일을 실행합니다.  
최적화된 경로, 장애물 회피 거동, 로봇의 자세 등을 그래프와 애니메이션으로 확인할 수 있습니다.


---
## ❓ 자주 묻는 질문 (FAQ)
Q. Julia 실행 중 zsh: killed 에러가 발생해요.

A. 메모리 부족이나 패키지 충돌 문제입니다. Manifest.toml 파일을 삭제한 후, 2. 초기 설정(Instantiate) 단계를 다시 진행해 보세요.

Q. MATLAB에서 설정한 장애물이 Julia에 반영되지 않아요.

A. Mission_Control.m을 실행하여 config_obstacles.csv 파일이 갱신되었는지 확인하세요. Julia는 이 CSV 파일을 읽어서 환경을 구축합니다.
