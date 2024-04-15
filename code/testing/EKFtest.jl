include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")
include("../estimators/EKF.jl")

using LinearAlgebra
using Plots
#--------------------------------------------  
# Dynamics and measurement Generation
#--------------------------------------------  

# Dynamics
Cd = 0.1
params = Params(.5, 100.0, Cd, .2)
x0 = [0.0, 50.0, 0.4, 15.0]
x_list, u_list = genTrajectory(x0, params)

# Measurements
function rNoise()
    return 0.0
    # return rand(Chisq(4))
end
function rdNoise()
    return 0.0
    #return rand(Normal(0,.2))
end
function azNoise()
    return 0.0
    # return rand(Normal(0.0, deg2rad(2)))
end
function elNoise()
    return 0.0
    # return rand(Normal(0.0, deg2rad(2)))
end
radar = Radar([50.0,0.0], rNoise, rdNoise, azNoise, elNoise)
y_list = radarMeasure(x_list, radar)

# init P
bignum = Diagonal([100, 100, 1, 50])
P0 = ones(4, 4)*bignum

# init Qk, Rk
Qk = Diagonal([5, 5, 0.1, 5])
Rk = Diagonal([10, 10, 0.3])


# print(length(y_list))
# print(length(u_list))
x_EKF, P_EKF = EKF_bulk(x0*1.3, P0, u_list, y_list, Qk, Rk, params, radar)

x_EKF = stack(x_EKF, dims=1)
xMat = stack(x_list, dims=1)

scatter(x_EKF[:,:2], label="EKF")
scatter!(xMat[:,:2], label="data")



# display(P_list)