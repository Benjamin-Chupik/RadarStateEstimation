using RadarStateEstimation
using RadarStateEstimation.problemStruct # This exports Parmas

using LinearAlgebra
using Plots
using Distributions
#--------------------------------------------  
# Dynamics and measurement Generation
#--------------------------------------------  

# Dynamics
Cd = 0.1
params = Params(.5, 100.0, Cd, .2)
x0 = [0.0, 50.0, 0.0, 5.0]

Dynnoise = [Normal(0, 0.01), Normal(0, 0.01), Normal(0, 0.1), Normal(0, 10)] # Vector of Distributions for dynamics noise

x_list, u_list = RadarStateEstimation.models.fixedWing.genTrajectory(x0, params)

# Measurements
rNoise = Chisq(4)
rdNoise = Normal(0,.2)
elNoise = Normal(0.0, deg2rad(2))

radar = RadarStateEstimation.models.radar.Radar([50.0,0.0], rNoise, rdNoise, elNoise)
y_list = RadarStateEstimation.models.radar.radarMeasure(x_list, radar)
y_list_noNoise = RadarStateEstimation.models.radar.radarMeasure_noNoise(x_list, radar)

xMat = stack(x_list, dims=1)
yMat = stack(y_list, dims=1)
yMat_noNoise = stack(y_list_noNoise, dims=1)


#--------------------------------------------  
# EKF Setup and Testing
#--------------------------------------------  

P0 = Diagonal([50, 50, 0.3, 20])
x_init = rand(MvNormal(x0, P0))

# init Qk, Rk
Qk = Diagonal([0.3, 0.3, 0.03, 0.3])
Rk = Diagonal([var(elNoise), var(rNoise), var(rdNoise)])

x_EKF, P_EKF = RadarStateEstimation.estimators.EKF.EKF_bulk(x_init, P0, 0*u_list, y_list, Qk, Rk, params, radar)
x_noisy = RadarStateEstimation.models.radar.y2p(y_list, radar)

x_EKF = stack(x_EKF, dims=1)
xMat = stack(x_list, dims=1)
xNoisy = stack(x_noisy, dims=1)

scatter(x_EKF[:,:2], label="EKF")
scatter!(xMat[:,:2], label="Truex")
scatter!(xNoisy[:,:2], label="Noisyx")



# display(P_list)