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
x0 = [50.0, 50.0, 0.3, 20.0]
x_list, u_list = genTrajectory(x0, params)

# Measurements
rNoise = Normal(0,.2) # Chisq(4)
rdNoise = Normal(0,.2)
elNoise = Normal(0.0, deg2rad(2))

radar = Radar([50.0,0.0], rNoise, rdNoise, elNoise)


y_list = radarMeasure(x_list, radar)

# init P
P0 = Diagonal([10, 10, 0.2, 5])

# init Qk, Rk
Qk = Diagonal([0.01, 0.01, 0.005, 0.01])
@show Rk = Diagonal([var(rNoise), var(rdNoise), var(elNoise)])


# print(length(y_list))
# print(length(u_list))
@show x_init = rand(MvNormal(x0, P0))
x_EKF, P_EKF = EKF_bulk(x_init, P0, u_list, y_list, Qk, Rk, params, radar)

x_EKF = stack(x_EKF, dims=1)
xMat = stack(x_list, dims=1)

scatter(x_EKF[:,:2], label="EKF")
scatter!(xMat[:,:2], label="data")



# display(P_list)