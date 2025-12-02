# 🚀 Astrobee SE2 Trajectory Optimization

이 프로젝트는 2D 환경에서 **Astrobee 로봇**이 장애물을 피해 목표 지점까지 이동하는 **최적 경로(Trajectory)**를 계산하는 시뮬레이션입니다.

**SCP (Sequential Convex Programming)** 알고리즘을 사용하여 사용자가 설정한 환경(장애물 위치, 크기 등)에 따라 충돌 없는 경로를 생성합니다.

---

## 🛠️ 1. 필수 프로그램 설치 (Prerequisites)

> **중요:** 이 코드는 패키지 호환성을 위해 반드시 **Julia 1.7.1** 버전에서 실행해야 합니다.  
> 최신 Julia(1.10 등)에서는 패키지 충돌로 인해 실행되지 않습니다.

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

---

## ✔️ Step 2 — Julia 1.7.1 설치 및 기본 버전 설정

```bash
juliaup add 1.7.1
juliaup default 1.7.1
julia --version
```

---

# ⚙️ 2. 프로젝트 설정 (Installation)

```bash
cd 경로/Astrobee_Project
```

### 패키지 자동 설치

```bash
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

---

# 🖥️ 3. 실행 방법 (How to Run)

```bash
julia main.jl
```

---

# 🕹️ 4. 환경 설정 변경 (User Configuration)

```julia
function get_config()
    return AstrobeeConfig(
        40, 41.0,
        [-1.0, -1.0], [1.0, 1.0],
        [
            (0.7,  0.7, 0.25, 0.10),
            (0.1,  0.0, 0.10, 0.25),
            (-0.5, 0.5, 0.20, 0.20)
        ],
        [-0.25, 0.4, 0.0, 0.0, 0.0, 0.0],
        [ 0.7, -0.5, 0.0, 0.0, 0.0, 0.0]
    )
end
```

---

# 📊 5. MATLAB 시각화

1. MATLAB 실행  
2. 프로젝트 폴더로 이동  
3. `trajectory.m` 실행  

---

# ❓ 문제 해결 (Troubleshooting)

### SystemError / Precompilation Error

```bash
julia --version
```

### Manifest.toml 충돌 해결

```bash
rm Manifest.toml
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

---
