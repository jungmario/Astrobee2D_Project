using SCPToolbox
using LinearAlgebra
using JuMP
using ECOS
using Printf
using Test

# 1. run_trials 함수 (솔버 실행 반복기)
# -----------------------------------------------------
function run_trials(
    mdl,
    traj::TrajectoryProblem,
    pars::T,
    solver::Module;
    num_trials::Int = 100,
) where {T<:SCPParameters}

    sol_list = Vector{SCPSolution}(undef, num_trials)
    history_list = Vector{SCPHistory}(undef, num_trials)

    for trial = 1:num_trials
        # 1. 문제 생성
        local pbm = solver.create(pars, traj)
        
        @printf("Trial %d/%d\n", trial, num_trials)
        
        # 2. 출력 숨기기 (첫 번째 이후)
        if trial > 1
            real_stdout = stdout
            (rd, wr) = redirect_stdout()
        end
        
        # 3. 솔버 실행
        sol_list[trial], history_list[trial] = solver.solve(pbm)
        
        # 4. 성공 여부 검사
        # (여기서 에러가 나면 파라미터 튜닝이 필요한 상태입니다)
        if sol_list[trial].status != "SCP_SOLVED" && sol_list[trial].status != "OPTIMAL"
            @warn "Trial $trial failed with status: $(sol_list[trial].status)"
        end
        
        if trial > 1
            redirect_stdout(real_stdout)
        end
    end

    # 마지막 결과 반환
    return sol_list[end], history_list[end]
end

# 2. gusto 함수 (메인 함수)
# -----------------------------------------------------
function gusto(config::AstrobeeConfig)

    # (1) 문제 정의 (Config 적용)
    mdl = Astrobee2DProblem(config) 
    pbm = TrajectoryProblem(mdl)
    define_problem!(pbm, :gusto)

    # (2) GuSTO 파라미터 설정 (UserConfig의 N 사용)
    N = config.N
    
    # [파라미터 튜닝 값]
    Nsub = 15
    iter_max = 50
    disc_method = FOH
    
    λ_init = 10.0
    λ_max = 1.0e10
    
    ρ_0 = 0.01
    ρ_1 = 0.3
    
    β_sh = 2.0
    β_gr = 2.0
    γ_fail = 2.0
    
    η_init = 2.0
    η_lb = 1e-3
    η_ub = 10.0
    μ = 1.0
    iter_μ = 50
    
    ε_abs = 1e-2
    ε_rel = 1e-2
    feas_tol = 1e-3
    
    pen = :quad
    hom = 1.0
    q_tr = Inf
    q_exit = Inf
    
    solver = ECOS
    solver_options = Dict("verbose" => 0)

    pars = GuSTO.Parameters(
        N, Nsub, iter_max, disc_method, 
        λ_init, λ_max, 
        ρ_0, ρ_1, 
        β_sh, β_gr, γ_fail, 
        η_init, η_lb, η_ub, 
        μ, iter_μ, 
        ε_abs, ε_rel, feas_tol, 
        pen, hom, q_tr, q_exit, 
        solver, solver_options
    )

    # (3) 실행
    sol, history = run_trials(mdl, pbm, pars, GuSTO; num_trials = 1)
    
    return mdl, sol, history
end