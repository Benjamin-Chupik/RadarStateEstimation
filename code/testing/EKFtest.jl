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
elNoise = Normal(0.0, deg2rad(2))
rNoise = Chisq(400) #Normal(0,.2) #
rdNoise = Normal(0,.2)


radar = Radar([50.0,0.0], rNoise, rdNoise, elNoise)
y_list = radarMeasure(x_list, radar)
x_noisy = y2p(y_list, radar)


P0 = Diagonal([50, 50, 0.3, 20])
x_init = rand(MvNormal(x0, P0))

# init Qk, Rk
Qk = Diagonal([0.5, 0.5, 0.01, 0.5])
@show Rk = Diagonal([var(elNoise), var(rNoise), var(rdNoise)])


x_EKF, P_EKF = EKF_bulk(x_init, P0, u_list, y_list, Qk, Rk, params, radar)

x_EKF = stack(x_EKF, dims=1)
xMat = stack(x_list, dims=1)
xNoisy = stack(x_noisy, dims=1)

scatter(x_EKF[:,:2], label="EKF")
scatter!(xMat[:,:2], label="Truex")
scatter!(xNoisy[:,:2], label="Noisyx")



# display(P_list)