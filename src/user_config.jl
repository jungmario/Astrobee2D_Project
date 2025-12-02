# src/user_config.jl
module UserConfig

using LinearAlgebra
using DelimitedFiles
export AstrobeeConfig, get_config

struct AstrobeeConfig
    N::Int
    tf::Float64
    room_min::Vector{Float64}
    room_max::Vector{Float64}
    obstacles::Vector{Tuple{Float64, Float64, Float64, Float64}}
    start_state::Vector{Float64}
    goal_state::Vector{Float64}
    waypoints::Vector{Tuple{Float64, Float64}}
end

function get_config()
    # 1. 파라미터 읽기
    # (파일이 없으면 기본값 사용하도록 예외처리)
    if isfile("config_params.csv")
        params = readdlm("config_params.csv", ',')
        N = Int(params[1])
        tf = Float64(params[2])
    else
        N, tf = 40, 41.0
    end

    if isfile("config_room.csv")
        room_data = readdlm("config_room.csv", ',')
        room_min = Float64.(room_data[1, :])
        room_max = Float64.(room_data[2, :])
    else
        room_min, room_max = [-5.0, -5.0], [5.0, 5.0]
    end

    obstacles = Vector{Tuple{Float64, Float64, Float64, Float64}}()
    if isfile("config_obstacles.csv")
        obs_data = readdlm("config_obstacles.csv", ',')
        if ndims(obs_data) == 1
            push!(obstacles, (obs_data[1], obs_data[2], obs_data[3], obs_data[4]))
        else
            for i in 1:size(obs_data, 1)
                push!(obstacles, (obs_data[i,1], obs_data[i,2], obs_data[i,3], obs_data[i,4]))
            end
        end
    else
        # 기본 장애물
        push!(obstacles, (0.7, 0.7, 0.25, 0.10))
        push!(obstacles, (0.1, 0.0, 0.10, 0.25))
        push!(obstacles, (0.0, -0.5, 0.17, 0.17))
    end

    if isfile("config_state.csv")
        state_data = readdlm("config_state.csv", ',')
        start_state = Float64.(state_data[1, :])
        goal_state = Float64.(state_data[2, :])
    else
        start_state = [-4.9, -4.9, 0.0, 0.0, 0.0, 0.0]
        goal_state = [4.9, 4.9, 0.0, 0.0, 0.0, 0.0]
    end

    # [핵심 수정] waypoints 변수를 try 밖에서 미리 만듭니다!
    waypoints = Vector{Tuple{Float64, Float64}}() 
    
    try
        if isfile("config_waypoints.csv")
            wp_data = readdlm("config_waypoints.csv", ',')
            # 내용이 "empty" 문자열이거나 비어있는지 체크
            if wp_data == "empty" || isempty(wp_data)
                # Do nothing (keep empty)
            elseif ndims(wp_data) == 1
                push!(waypoints, (wp_data[1], wp_data[2]))
            else
                for i in 1:size(wp_data, 1)
                    push!(waypoints, (wp_data[i,1], wp_data[i,2]))
                end
            end
        end
    catch e
        # 읽기 실패 시 그냥 빈 경로로 진행
        println("Waypoints load failed: $e")
    end

    return AstrobeeConfig(
        N, tf,
        room_min, room_max,
        obstacles,
        start_state, goal_state,
        waypoints
    )
end

end