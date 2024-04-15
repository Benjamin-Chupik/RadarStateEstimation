include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")
include("../estimators/EKF.jl")

## setup
Cd = 0.1
dt = 0.5
maxT = 10.0
ρ = 1.0
params = Params(dt, maxT, Cd, ρ)

# init x0, u0
x0 = [0.0, 0.0, 0.0, 5.0]
uk = [0.0, 0.0]

# init P
bignum = 1000
P0 = ones(4, 4)*bignum

# init Qk, Rk
Qk = zeros(4, 4)
Rk = zeros(3,3)

# init radar
function rNoise()
    return 0.0
end
function rdNoise()
    return 0.0
end
function azNoise()
    return 0.0
end
function elNoise()
    return 0.0
end
radar = Radar([50.0,0.0], rNoise, rdNoise, azNoise, elNoise)

# EKF
# println(x_list)
# display(P_list)

EKF_step!(x_list, P_list, uk, [2,2,2], Qk, Rk, params, radar)

println(x_list)
# display(P_list)