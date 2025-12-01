# src/user_config.jl

module UserConfig

using LinearAlgebra
export AstrobeeConfig, get_config

# 사용자가 설정할 모든 파라미터를 담는 구조체
struct AstrobeeConfig
    N::Int
    tf::Float64
    room_min::Vector{Float64}
    room_max::Vector{Float64}
    
    # [수정] 장애물: (x, y, radius) -> (x, y, radius_x, radius_y)
    obstacles::Vector{Tuple{Float64, Float64, Float64, Float64}}
    
    start_state::Vector{Float64}
    goal_state::Vector{Float64}
end

# =========================================================
# [사용자 수정 구역] 아래 함수 내부의 값들을 마음대로 수정하세요!
# =========================================================
function get_config()
    return AstrobeeConfig(
        # 1. 타임 스텝 & 시간
        40,     # N (스텝 수)
        41.0,   # tf (총 시간, 초)

        # 2. ISS 방 크기 (테이블 크기)
        [-1.0, -1.0],  # 최소 좌표 [x, y]
        [ 1.0,  1.0],  # 최대 좌표 [x, y]

        # [수정] (x, y, rx, ry) 형태로 입력
        # 예시: 첫 번째는 가로로 긴 타원, 두 번째는 세로로 긴 타원
        [
            (0.7,  0.7, 0.25, 0.10),  # 장애물 1 (가로로 길쭉)
            (0.1,  0.0, 0.10, 0.25),  # 장애물 2 (세로로 길쭉)
            (0.0, -0.5, 0.17, 0.17),   # 장애물 3 (그냥 원)
            (-0.5,1.0,0.10,0.10) # 장애물 4 (그냥 원)
        ],

        # 4. 시작 상태 [x, y, θ, vx, vy, ω]
        [-0.25,  0.4, 0.0, 0.0, 0.0, 0.0],

        # 5. 목표 상태 [x, y, θ, vx, vy, ω]
        [ 0.7,  -0.5, 0.0, 0.0, 0.0, 0.0]
    )
end

end