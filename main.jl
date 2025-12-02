using Pkg
Pkg.activate(".")
Pkg.instantiate()

using SCPToolbox
using DelimitedFiles

include("src/user_config.jl")
using .UserConfig

include("src/parameters.jl")
include("src/definition.jl")
include("src/solver.jl")

println(">>> 최적화 시작! (N=$(get_config().N))")
config = get_config()
(mdl, sol, history) = gusto(config)

if sol.status == "SCP_SOLVED" || sol.status == "OPTIMAL"
    println("\n>>> 🎉 성공! 결과 저장 중...")
    
    # 1. 궤적 데이터 저장 (기존 동일)
    data = hcat(sol.td, sol.xd', sol.ud')
    header = ["t" "rx" "ry" "theta" "vx" "vy" "omega" "Fx" "Fy" "M"]
    writedlm("astrobee_result.csv", [header; data], ',')
    println(">>> 궤적 저장 완료: astrobee_result.csv")
    
    # 2. [추가] 환경 설정(장애물) 저장
    # [수정] 환경(장애물) 저장: 타원 정보(rx, ry) 포함
    obs_list = config.obstacles
    # N x 4 행렬 (x, y, rx, ry)
    obs_matrix = zeros(length(obs_list), 4) 
    
    for (i, obs) in enumerate(obs_list)
        obs_matrix[i, :] = [obs[1], obs[2], obs[3], obs[4]]
    end
    
    # 헤더에 rx, ry 명시
    obs_header = ["ox" "oy" "rx" "ry"]
    writedlm("astrobee_config.csv", [obs_header; obs_matrix], ',')
    println(">>> 환경 저장 완료: astrobee_config.csv")

    # [추가] 방 크기 정보(Room Size) 저장
    # 1x4 행렬: [min_x, min_y, max_x, max_y]
    room_data = [config.room_min[1] config.room_min[2] config.room_max[1] config.room_max[2]]
    room_header = ["min_x" "min_y" "max_x" "max_y"]
    
    writedlm("astrobee_room.csv", [room_header; room_data], ',')
    println(">>> 방 크기 저장 완료: astrobee_room.csv")
    
else
    println("\n>>> ㅠㅠ 실패 상태: $(sol.status)")
end