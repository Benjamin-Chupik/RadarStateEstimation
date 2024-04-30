using RadarStateEstimation
using RadarStateEstimation.problemStruct # This exports Parmas
using RadarStateEstimation.estimators.SIR
import RadarStateEstimation.models.fixedWing as physMod
import RadarStateEstimation.models.radar as radMod

using Distributions
using Plots

"""
This will test the SIR Particle filter with the fixed wing dynamics to see if it works
"""


#--------------------------------------------  
# Dynamics and measurement Generation
#--------------------------------------------  
# Dynamics
Cd = .1
params = Params(.5, 100.0, Cd, 1.0)
x0 = [0.0, 50.0, 0.0, 25.0, 0.0] # [x, z, alpha, v, w]
x_list, u_list, w_list = physMod.genTrajectory(x0, params)

# Measurements
rNoise = Chisq(1)
rdNoise = Normal(0,.2)
elNoise = Normal(0.0, deg2rad(2))

radar = radMod.Radar([50.0,0.0], rNoise, rdNoise, elNoise)
y_list = radMod.radarMeasure(x_list, radar)
y_list_noNoise = radMod.radarMeasure_noNoise(x_list, radar)

xMat = stack(x_list, dims=1)
yMat = stack(y_list, dims=1)
yMat_noNoise = stack(y_list_noNoise, dims=1)


#--------------------------------------------  
# SIR Testing
#-------------------------------------------- 
# Other setup
Ns = 5000 # Number of prticles to use
dynamUp(x::Vector{Float64}, dt::Float64) = SIR.dynamicsUpdate(x, params)
pygx(y::Vector{Float64}, x::Vector{Float64}) = radMod.likelihood(y, x, radar)

# Set up initial state particles
x0s = Vector{Vector{Float64}}()
dx = 10
dz = 10
dα = deg2rad(10)
dv = 3
dw = 0
for i in 1:Ns
    x = Vector{Float64}(undef, 5)
    x[1] = rand(Uniform(x0[1]-dx, x0[1]+dx)) # x
    x[2] = rand(Uniform(x0[2]-dz, x0[2]+dz)) # z
    x[3] = rand(Uniform(x0[3]-dα, x0[3]+dα)) # α
    x[4] = rand(Uniform(x0[4]-dv, x0[4]+dv)) # v
    x[5] = 0.0 # w
    push!(x0s, x)
end

w0s = ones(Ns)./Ns # Initial weights (uniform)

pflt = plot(title = "Particle Filter Test")

xk = deepcopy(x0s)
for y in y_list
    SIR.SIR_update!(xk, w0s, y, params, dynamUp, pygx)
    scatter!(SIR.MMSE(xk))
    display(pflt)
end


