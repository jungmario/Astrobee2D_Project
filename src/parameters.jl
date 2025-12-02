# src/parameters.jl
using LinearAlgebra
using SCPToolbox
using .UserConfig

# ..:: Data structures ::..

struct Astrobee2DParameters
    id_r::IntRange; id_v::IntRange; id_θ::Int; id_ω::Int
    id_T::IntRange; id_M::Int; id_t::Int; id_δ::IntRange
    v_max::Real; ω_max::Real; T_max::Real; M_max::Real; m::Real; J::Real
end

struct Astrobee2DEnvironmentParameters
    obs::Vector{Ellipsoid}; iss::Vector{Hyperrectangle}
    n_obs::Int; n_iss::Int
end

mutable struct Astrobee2DTrajectoryParameters
    r0::RealVector; rf::RealVector; v0::RealVector; vf::RealVector
    θ0::Real; θf::Real; ω0::Real; ωf::Real
    tf_min::Real; tf_max::Real; γ::Real; hom::Real; ε_sdf::Real
    
    # [복구] 경유지 저장 필드
    waypoints::Vector{Vector{Float64}}
end

struct Astrobee2DProblem
    vehicle::Astrobee2DParameters
    env::Astrobee2DEnvironmentParameters
    traj::Astrobee2DTrajectoryParameters
end

# ..:: Methods ::..

function Astrobee2DEnvironmentParameters(iss, obs)
    return Astrobee2DEnvironmentParameters(obs, iss, length(obs), length(iss))
end

function Astrobee2DProblem(config::AstrobeeConfig)::Astrobee2DProblem

    # 1. 환경 설정 (타원형 장애물 지원)
    r_robot = 0.215
    clearance = 0.05
    
    obs = Vector{Ellipsoid}()
    
    # [수정] (x, y, rx, ry) 4개 변수로 받기
    for (ox, oy, rx_obs, ry_obs) in config.obstacles
        # 가로/세로 각각 팽창 (Inflation)
        rx_safe = rx_obs + r_robot + clearance
        ry_safe = ry_obs + r_robot + clearance
        
        # 타원 행렬 생성
        H_obs = diagm([1.0/rx_safe, 1.0/ry_safe])
        push!(obs, Ellipsoid(H_obs, [ox; oy]))
    end

    iss_rooms = [Hyperrectangle(config.room_min, config.room_max)]
    env = Astrobee2DEnvironmentParameters(iss_rooms, obs)

    # 2. 로봇 파라미터
    id_r, id_θ = 1:2, 3
    id_v, id_ω = 4:5, 6
    id_T, id_M = 1:2, 3
    id_t = 1
    id_δ = (1:(config.N * env.n_iss)) .+ 1

    mass = 14.4; J = 0.1083
    T_max = mass * 0.02; M_max = J * (10 * π / 180)
    v_max = 0.20; ω_max = 10 * π / 180

    veh = Astrobee2DParameters(id_r, id_v, id_θ, id_ω, id_T, id_M, id_t, id_δ, v_max, ω_max, T_max, M_max, mass, J)

    # 3. 궤적 파라미터 (경유지 포함)
    # [수정] Tuple 리스트 -> Vector 리스트 변환
    wp_data = [collect(wp) for wp in config.waypoints]

    traj = Astrobee2DTrajectoryParameters(
        config.start_state[1:2], config.goal_state[1:2],
        config.start_state[4:5], config.goal_state[4:5],
        config.start_state[3], config.goal_state[3],
        config.start_state[6], config.goal_state[6],
        config.tf, config.tf, 0.1, 50.0, 1e-4,
        wp_data # [복구] 경유지 전달
    )

    return Astrobee2DProblem(veh, env, traj)
end