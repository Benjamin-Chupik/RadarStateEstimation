using LinearAlgebra

include("../problemStruct.jl")
include("../models/fixedWing.jl")
# include("../models/radarMeasure.jl")


function EKF_step!(x_list, P_list, uk, yk1, Qk, Rk, params)
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

    yk1hat = [1,1,1] #h(xk1min) #TODO
    Hk1 = ones(3,4) #genH(xk1min) #TODO
    ek1 = yk1 .- yk1hat
    
    Kk1 = Pk1min*Hk1'*inv(Hk1*Pk1min*Hk1'+ Rk)

    xk1plus = xk1min + Kk1*ek1
    
    Pk1plus = (I-Kk1*Hk1)*Pk1min
    
    push!(x_list, xk1plus)
    push!(P_list, Pk1plus)
end

## setup
Cd = 0.1
dt = 0.5
maxT = 10.0
ρ = 1.0
params = Params(dt, maxT, Cd, ρ)

x0 = [0.0, 0.0, 0.0, 5.0]
x_list = Vector{Vector{Float64}}()
push!(x_list, x0)
uk = [0.0, 0.0]

bignum = 1000
P_list = Vector{Matrix{Float64}}()
P0 = ones(4, 4)*bignum
push!(P_list, P0)

Qk = ones(4, 4)
Rk = [1.0 0 0
      0 1.0 0
      0 0 1.0]

# EKF
println(x_list)
display(P_list)

EKF_step!(x_list, P_list, uk, [2,2,2], Qk, Rk, params)

println(x_list)
display(P_list)