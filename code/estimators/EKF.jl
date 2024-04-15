using LinearAlgebra

include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")


function EKF_step!(x_list, P_list, uk, yk1, Qk, Rk, params, radar)
    # n = length(xk)
    # xk = x_list[end]
    # Pk = P_list[end]

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
