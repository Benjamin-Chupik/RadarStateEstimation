using LinearAlgebra

include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")


function EKF_step!(x_list, P_list, uk, yk1, Qk, Rk, params, radar)
    """
    Updates the x_list with xk+1 with an EKF prediction
    Inputs:
        x_list: list containing the x vectors so far
        P_list: list containing the P matricies so far
        uk: control input corresponding to timestep k
        yk1: measurement corresponding to timestep k+1
        Qk: EKF dynamics noise at step k
        Rk: EKF measurement noise at step k
        params: paramter struct
        radar: radar struct
    Updates:
        x_list with new x
        P_list with new P
    """

    ############### PREDICTION STEP ################
    xk = x_list[end]
    Pk = P_list[end]
    
    xk1min = simulate(xk, uk, params) # nonlinear dynamics prediction
    
    Atild = fixedWingLinDyn(xk, params) # linearized dynamics at timestep
    Fk = I + params.dt*Atild 

    Pk1min = Fk*Pk*Fk' + Qk # update Pmin

    yk1hat = radarMeasure(xk1min, radar)
    Hk1 = fixedWingMeasDer(xk1min, radar)

    ek1 = yk1 .- yk1hat
    
    Kk1 = Pk1min*Hk1'*inv(Hk1*Pk1min*Hk1'+ Rk)

    xk1plus = xk1min + Kk1*ek1
    
    Pk1plus = (I-Kk1*Hk1)*Pk1min
    
    push!(x_list, xk1plus)
    push!(P_list, Pk1plus)
end

function EKF_bulk(x0, P0, U, Y, Qk, Rk, params, radar)
    """
    Updates the x_list with xk+1 with an EKF prediction
    Inputs:
        x_list: vector x0
        P_list: matrix P0
        U: Vector of control imputs for all k
        Y: Vector of measurements for all k (k=1 should be empty)
        Qk: EKF dynamics noise at step k
        Rk: EKF measurement noise at step k
        params: paramter struct
        radar: radar struct
    Updates:
        x_list with all x
        P_list with all P
    """
    # @assert length(U) == length(Y)
    K = length(U)

    # Initialize x_list
    x_list = Vector{Vector{Float64}}()
    push!(x_list, x0)

    # Initialize P_list
    P_list = Vector{Matrix{Float64}}()
    push!(P_list, P0)

    for k in 1:K
        println("k=$(k)")
        EKF_step!(x_list, P_list, U[k], Y[k+1], Qk, Rk, params, radar)
    end

    return (x_list, P_list)
end
