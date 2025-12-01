"""
6-Degree of Freedom free-flyer problem definition.

Sequential convex programming algorithms for trajectory optimization.
Copyright (C) 2021 Autonomous Controls Laboratory (University of Washington),
                   and Autonomous Systems Laboratory (Stanford University)

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <https://www.gnu.org/licenses/>.
"""

using JuMP
using ECOS
using Printf

# ..:: Methods ::..

function define_problem!(pbm::TrajectoryProblem, algo::Symbol)::Nothing
    set_dims!(pbm)
    set_scale!(pbm)
    set_integration!(pbm)
    set_cost!(pbm, algo)
    set_dynamics!(pbm)
    set_convex_constraints!(pbm, algo)
    set_nonconvex_constraints!(pbm, algo)
    set_bcs!(pbm)
    set_guess!(pbm)

    return nothing
end

function set_dims!(pbm::TrajectoryProblem)::Nothing

    # 파라미터 개수 가져오기 (시간 + 델타 변수들)
    # parameters.jl에서 정의한 id_δ의 마지막 번호가 전체 개수입니다.
    np = pbm.mdl.vehicle.id_δ[end]

    # 문제 차원 설정: (pbm, nx, nu, np)
    # nx=6 (2D 상태), nu=3 (2D 입력)
    problem_set_dims!(pbm, 6, 3, np)

    return nothing
end

function set_scale!(pbm::TrajectoryProblem)::Nothing

    mdl = pbm.mdl
    veh, traj, env = mdl.vehicle, mdl.traj, mdl.env # [추가] env 가져오기
    
    # 1. 위치(Pos) 스케일링: 환경에 설정된 방 크기 사용
    # iss[1]은 우리가 parameters.jl에서 만든 첫 번째 방(Hyperrectangle)입니다.
    # Hyperrectangle.l (lower bound)과 .u (upper bound) 필드에 최소/최대 좌표가 들어있습니다.
    
    # 방이 없는 경우를 대비해 안전장치를 둘 수도 있지만, Astrobee 문제는 방이 무조건 1개 있다고 가정합니다.
    room = env.iss[1]
    world_min = room.l  # [min_x, min_y]
    world_max = room.u  # [max_x, max_y]

    for (idx, i) in enumerate(veh.id_r)
        problem_advise_scale!(pbm, :state, i, (world_min[idx], world_max[idx]))
    end
    
    # 2. 각도(Theta) 스케일링: -pi ~ pi
    problem_advise_scale!(pbm, :state, veh.id_θ, (-Float64(pi), Float64(pi)))
    
    # 3. 속도(Vel) 스케일링
    for i in veh.id_v
        problem_advise_scale!(pbm, :state, i, (-veh.v_max, veh.v_max))
    end
    
    # 4. 각속도(Omega) 스케일링
    problem_advise_scale!(pbm, :state, veh.id_ω, (-veh.ω_max, veh.ω_max))

    # 5. 시간(Time) 파라미터 스케일링
    problem_advise_scale!(pbm, :parameter, veh.id_t, (traj.tf_min, traj.tf_max))

    # 6. 방 로직(Delta) 파라미터 스케일링
    if isdefined(veh, :id_δ)
        for i in veh.id_δ
            problem_advise_scale!(pbm, :parameter, i, (-100.0, 1.0))
        end
    end

    return nothing
end

function set_integration!(pbm::TrajectoryProblem)::Nothing

    return nothing
end

# test/examples/astrobee_se2/definition.jl

function set_guess!(pbm::TrajectoryProblem)::Nothing

    problem_set_guess!(
        pbm,
        (N, pbm) -> begin
            # 1. 데이터 가져오기
            veh = pbm.mdl.vehicle
            env = pbm.mdl.env
            traj = pbm.mdl.traj

            # 2. 파라미터(p) 및 시간 초기화
            p = zeros(pbm.np)
            # 시간 고정 문제 (tf = 41.0)
            flight_time = 0.5 * (traj.tf_min + traj.tf_max)
            p[veh.id_t] = flight_time

            # 3. 상태(x) 초기화: 직선 경로 (Straight Line)
            x = zeros(pbm.nx, N)

            # (1) 위치 (r): 시작점(r0)에서 끝점(rf)까지 직선 긋기
            # straightline_interpolate는 SCPToolbox 내장 함수
            x[veh.id_r, :] = straightline_interpolate(traj.r0, traj.rf, N)

            # (2) 속도 (v): 평균 속도로 초기화 (중요!)
            # 위치는 변하는데 속도가 0이면 물리적으로 말이 안 되어서 솔버가 힘들어합니다.
            # v = (끝점 - 시작점) / 비행시간
            avg_vel = (traj.rf - traj.r0) / flight_time
            
            # 모든 스텝에 평균 속도 할당
            for k = 1:N
                x[veh.id_v, k] = avg_vel
            end

            # (3) 각도(θ) 및 각속도(ω): 0으로 초기화
            # 회전은 충돌과 무관하므로 0으로 둬도 됩니다.
            x[veh.id_θ, :] .= 0.0
            x[veh.id_ω, :] .= 0.0

            # 4. 방 로직(δ) 초기화 (기존 코드 유지)
            if isdefined(veh, :id_δ)
                r = view(x, veh.id_r, :) 
                δ = reshape(view(p, veh.id_δ), env.n_iss, N)
                for i = 1:env.n_iss
                    roomi = env.iss[i] 
                    for k = 1:N
                        # 2D 거리 계산
                        relative_pos = (r[:, k] - roomi.c) ./ roomi.s
                        dist = norm(relative_pos, Inf)
                        δ[i, k] = 1.0 - dist
                    end
                end
            end

            # 5. 입력(u) 초기화: 0으로 시작
            u = zeros(pbm.nu, N)

            return x, u, p
        end,
    )

    return nothing
end

function set_cost!(pbm::TrajectoryProblem, algo::Symbol)::Nothing

    # 1. Terminal cost (최종 비용)
    # -----------------------------------------------------
    # 시간 고정 & 제약조건 처리로 인해 최종 비용 없음
    problem_set_terminal_cost!(
        pbm,
        (x, p, pbm) -> 0.0
    )

    # 2. Running cost (경로 비용)
    # -----------------------------------------------------
    # GuSTO 알고리즘 전용 (이차항 행렬 S 반환)
    # SCvx 코드는 과감하게 삭제했습니다.
    
    if algo == :gusto
        problem_set_running_cost!(
            pbm,
            algo,
            (t, k, p, pbm) -> begin
                veh = pbm.mdl.vehicle
                
                T_max_sq = veh.T_max^2
                M_max_sq = veh.M_max^2
                
                S = zeros(pbm.nu, pbm.nu)
                
                # [Fx, Fy] 비용 (단위행렬 2x2)
                S[veh.id_T, veh.id_T] = (1.0 / T_max_sq) * I(2)
                
                # [Mz] 비용 (스칼라)
                S[veh.id_M, veh.id_M] = 1.0 / M_max_sq
                
                return S
            end,
        )
    end

    return nothing
end

function set_dynamics!(pbm::TrajectoryProblem)::Nothing

    problem_set_dynamics!(
        pbm,
        # 1. Dynamics f (물리 법칙)
        # -------------------------------------------------
        # dot{x} = f(x, u) * tdil
        (t, k, x, u, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            tdil = p[veh.id_t] # Time dilation (비행 시간)

            # 상태(x)와 입력(u) 추출
            v = x[veh.id_v]      # 속도 [vx, vy]
            ω = x[veh.id_ω]      # 각속도 ω (Scalar)
            T = u[veh.id_T]      # 추력 [Fx, Fy]
            M = u[veh.id_M]      # 토크 M (Scalar)

            # 미분식 계산 (초기화)
            f = zeros(pbm.nx)
            
            # [Kinematics] 위치 미분 = 속도, 각도 미분 = 각속도
            f[veh.id_r] = v
            f[veh.id_θ] = ω

            # [Dynamics] 속도 미분 = 가속도(F/m), 각속도 미분 = 각가속도(M/J)
            f[veh.id_v] = T / veh.m
            f[veh.id_ω] = M / veh.J

            # 전체 식에 시간(tdil) 곱하기
            f *= tdil
            return f
        end,

        # 2. Jacobian df/dx (A Matrix)
        # -------------------------------------------------
        # 상태가 변할 때 움직임이 어떻게 변하는가?
        (t, k, x, u, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            tdil = p[veh.id_t]

            A = zeros(pbm.nx, pbm.nx)
            
            # 위치 미분(속도)에 대한 자코비안 -> I(2)
            A[veh.id_r, veh.id_v] = I(2)
            
            # 각도 미분(각속도)에 대한 자코비안 -> 1.0 (스칼라)
            A[veh.id_θ, veh.id_ω] = 1.0

            A *= tdil
            return A
        end,

        # 3. Jacobian df/du (B Matrix)
        # -------------------------------------------------
        # 입력이 변할 때 움직임이 어떻게 변하는가?
        (t, k, x, u, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            tdil = p[veh.id_t]

            B = zeros(pbm.nx, pbm.nu)
            
            # 속도 미분(가속도)은 힘(T)에 비례 -> 1/m
            B[veh.id_v, veh.id_T] = (1.0 / veh.m) * I(2)
            
            # 각속도 미분(각가속도)은 토크(M)에 비례 -> 1/J
            B[veh.id_ω, veh.id_M] = 1.0 / veh.J

            B *= tdil
            return B
        end,

        # 4. Jacobian df/dp (F Matrix)
        # -------------------------------------------------
        # 파라미터(시간)가 변할 때 움직임이 어떻게 변하는가?
        (t, k, x, u, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            tdil = p[veh.id_t]
            
            F = zeros(pbm.nx, pbm.np)
            
            # 시간(tdil)에 대한 미분은 원래 물리식 f(x,u)와 같음
            # (tdil이 곱해지기 전의 순수 f를 구하기 위해 다시 나눠줌)
            F[:, veh.id_t] = pbm.f(t, k, x, u, p) / tdil
            
            return F
        end,
    )

    return nothing
end

function set_convex_constraints!(pbm::TrajectoryProblem, algo::Symbol)::Nothing

    # 1. 상태 변수 제약 (State Constraints)
    # -----------------------------------------------------
    problem_set_X!(
        pbm,
        (t, k, x, p, pbm, ocp) -> begin
            traj = pbm.mdl.traj
            veh = pbm.mdl.vehicle
            env = pbm.mdl.env
            common = (pbm, ocp, algo)

            # 변수 추출 (2D & Scalar)
            v = x[veh.id_v]      # 2D Vector [vx, vy]
            ω = x[veh.id_ω]      # Scalar [ω]
            tdil = p[veh.id_t]
            
            # (1) 선형 속도 제한 (2D Norm)
            # ||v|| <= v_max
            define_conic_constraint!(
                common...,
                SOC,
                "max_lin_vel",
                (v,),
                (v) -> vcat(veh.v_max, v),
            )

            # (2) 각속도 제한 (Scalar Absolute Value)
            # |ω| <= ω_max
            # 스칼라에 SOC를 걸면 절댓값 제약과 같습니다.
            define_conic_constraint!(
                common...,
                SOC,
                "max_ang_vel",
                (ω,),
                (ω) -> vcat(veh.ω_max, ω),
            )

            # (3) 시간 제한 (고정 시간이지만 형식상 유지)
            define_conic_constraint!(
                common...,
                NONPOS,
                "max_duration",
                (tdil,),
                (tdil) -> tdil - traj.tf_max,
            )

            define_conic_constraint!(
                common...,
                NONPOS,
                "min_duration",
                (tdil,),
                (tdil) -> traj.tf_min - tdil,
            )

            # (4) 비행 구역(Table) 제약 (Room SDFs)
            # id_δ를 사용하기로 했으므로 포함
            if isdefined(veh, :id_δ)
                δ = reshape(p[veh.id_δ], env.n_iss, :)
                
                for i = 1:env.n_iss
                    desc = "room_sdf_$(i)"
                    # room[i].c, room[i].s는 2D 벡터여야 함 (parameters.jl에서 확인)
                    define_conic_constraint!(
                        common...,
                        LINF,
                        desc,
                        (δ[i, k], x[veh.id_r]), # 위치 r = x[veh.id_r]
                        (δik, r) -> vcat(1 - δik, (r - env.iss[i].c) ./ env.iss[i].s),
                    )
                end
            end
        end,
    )

    # 2. 입력 변수 제약 (Input Constraints)
    # -----------------------------------------------------
    problem_set_U!(
        pbm,
        (t, k, u, p, pbm, ocp) -> begin
            veh = pbm.mdl.vehicle
            common = (pbm, ocp, algo)

            T = u[veh.id_T] # 2D Vector [Fx, Fy]
            M = u[veh.id_M] # Scalar [Mz]

            # (1) 추력 제한 (2D Norm)
            # ||T|| <= T_max (F = ma로 변환된 값)
            define_conic_constraint!(
                common...,
                SOC,
                "max_thrust",
                (T,),
                (T) -> vcat(veh.T_max, T),
            )

            # (2) 토크 제한 (Scalar Absolute Value)
            # |M| <= M_max
            define_conic_constraint!(
                common...,
                SOC,
                "max_torque",
                (M,),
                (M) -> vcat(veh.M_max, M),
            )
        end,
    )

    return nothing
end

function set_nonconvex_constraints!(pbm::TrajectoryProblem, algo::Symbol)::Nothing

    # 1. 제약 조건 함수 s(x) <= 0
    astro_s = (t, k, x, u, p, pbm) -> begin
        env = pbm.mdl.env
        veh = pbm.mdl.vehicle
        traj = pbm.mdl.traj

        r = x[veh.id_r] # 위치 [x, y]
        
        s = zeros(env.n_obs + 1)
        
        # (1) 장애물 회피 (단순 타원 식)
        # 로봇을 점으로 취급하고, 팽창된 장애물(obs)과 충돌 검사
        for i = 1:env.n_obs
            E = env.obs[i]
            # E(r) < 1 이면 충돌 (거리 < r_safe)
            # 1 - E(r) <= 0  =>  E(r) >= 1 (안전)
            s[i] = 1.0 - E(r)
        end
        
        # (2) 방 로직
        δ = reshape(p[veh.id_δ], env.n_iss, :)[:, k]
        s[end] = -logsumexp(δ; t = traj.hom)

        return s
    end

    # 2. Jacobian C (수식 미분)
    astro_C = (t, k, x, u, p, pbm) -> begin
        env = pbm.mdl.env
        veh = pbm.mdl.vehicle
        
        r = x[veh.id_r]
        C = zeros(env.n_obs + 1, pbm.nx)
        
        # 장애물 미분: s = 1 - E(r) => ds/dr = -∇E(r)
        for i = 1:env.n_obs
            E = env.obs[i]
            # 타원체 기울기 계산 (SCPToolbox 내장 함수)
            C[i, veh.id_r] = -∇(E, r)
        end
        
        return C
    end

    # 3. Jacobian G (기존 유지)
    astro_G = (t, k, x, u, p, pbm) -> begin
        veh = pbm.mdl.vehicle
        env = pbm.mdl.env
        traj = pbm.mdl.traj
        
        id_δ_k = reshape(veh.id_δ, env.n_iss, :)[:, k]
        δ = p[id_δ_k]
        G = zeros(env.n_obs + 1, pbm.np)
        E_eye = RealMatrix(I(env.n_iss))
        _, ∇d = logsumexp(δ, [E_eye[:, i] for i = 1:env.n_iss]; t = traj.hom)
        G[end, id_δ_k] = -∇d
        return G
    end

    if algo == :gusto
        problem_set_s!(pbm, algo, 
            (t, k, x, p, pbm) -> astro_s(t, k, x, nothing, p, pbm),
            (t, k, x, p, pbm) -> astro_C(t, k, x, nothing, p, pbm),
            (t, k, x, p, pbm) -> astro_G(t, k, x, nothing, p, pbm)
        )
    end

    # 5. Jacobian G
    astro_G = (t, k, x, u, p, pbm) -> begin
        veh = pbm.mdl.vehicle
        env = pbm.mdl.env
        traj = pbm.mdl.traj
        
        id_δ_k = reshape(veh.id_δ, env.n_iss, :)[:, k]
        δ = p[id_δ_k]
        G = zeros(env.n_obs + 1, pbm.np)
        E_eye = RealMatrix(I(env.n_iss))
        _, ∇d = logsumexp(δ, [E_eye[:, i] for i = 1:env.n_iss]; t = traj.hom)
        G[end, id_δ_k] = -∇d
        return G
    end

    if algo == :gusto
        problem_set_s!(pbm, algo, 
            (t, k, x, p, pbm) -> astro_s(t, k, x, nothing, p, pbm),
            (t, k, x, p, pbm) -> astro_C(t, k, x, nothing, p, pbm),
            (t, k, x, p, pbm) -> astro_G(t, k, x, nothing, p, pbm)
        )
    end
    return nothing
end

function set_bcs!(pbm::TrajectoryProblem)::Nothing

    # 1. 초기 조건 (Initial Conditions)
    # -----------------------------------------------------
    # x(0) - x_start = 0
    problem_set_bc!(
        pbm,
        :ic,
        # Constraint g
        (x, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            traj = pbm.mdl.traj
            
            # 목표값 벡터 생성 (0으로 초기화)
            rhs = zeros(pbm.nx)
            
            # 값 채워넣기
            rhs[veh.id_r] = traj.r0   # 위치 [x, y]
            rhs[veh.id_v] = traj.v0   # 속도 [vx, vy]
            rhs[veh.id_θ] = traj.θ0   # [수정] 각도 θ (Scalar)
            rhs[veh.id_ω] = traj.ω0   # [수정] 각속도 ω (Scalar)
            
            # g = 현재 상태 - 목표 상태 = 0이어야 함
            g = x - rhs
            return g
        end,
        # Jacobian dg/dx -> Identity Matrix (1)
        (x, p, pbm) -> begin
            return I(pbm.nx)
        end,
        # Jacobian dg/dp -> Zero Matrix (0)
        (x, p, pbm) -> begin
            return zeros(pbm.nx, pbm.np)
        end,
    )

    # 2. 종단 조건 (Terminal Conditions)
    # -----------------------------------------------------
    # x(tf) - x_goal = 0
    problem_set_bc!(
        pbm,
        :tc,
        # Constraint g
        (x, p, pbm) -> begin
            veh = pbm.mdl.vehicle
            traj = pbm.mdl.traj
            
            rhs = zeros(pbm.nx)
            
            rhs[veh.id_r] = traj.rf   # 목표 위치
            rhs[veh.id_v] = traj.vf   # 목표 속도
            rhs[veh.id_θ] = traj.θf   # [수정] 목표 각도
            rhs[veh.id_ω] = traj.ωf   # [수정] 목표 각속도
            
            g = x - rhs
            return g
        end,
        # Jacobian dg/dx
        (x, p, pbm) -> begin
            return I(pbm.nx)
        end,
        # Jacobian dg/dp
        (x, p, pbm) -> begin
            return zeros(pbm.nx, pbm.np)
        end,
    )

    return nothing
end