include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")
include("../models/basicKinematics.jl")
include("../estimators/SIR.jl")
using Plots

"""
This will test the SIR Particle filter with the fixed wing dynamics to see if it works
"""


#--------------------------------------------  
# Dynamics and measurement Generation
#--------------------------------------------  

# Dynamics
Cd = 1.0
params = Params(.5, 100.0, Cd, .2)
x0 = [0.0, 50.0, 0.0, 5.0]
x_list, u_list = genTrajectory(x0, params)

# Measurements
rNoise = Chisq(4)
rdNoise = Normal(0,.2)
elNoise = Normal(0.0, deg2rad(2))

radar = Radar([50.0,0.0], rNoise, rdNoise, elNoise)
y_list = radarMeasure(x_list, radar)
y_list_noNoise = radarMeasure_noNoise(x_list, radar)

xMat = stack(x_list, dims=1)
yMat = stack(y_list, dims=1)
yMat_noNoise = stack(y_list_noNoise, dims=1)



#--------------------------------------------  
# SIR Testing
#-------------------------------------------- 
# Other setup
Ns = 500 # Number of prticles to use
dynamUp(x::Vector{Float64}, dt::Float64) = simulate(x, dt)
pygx(y::Vector{Float64}, x::Vector{Float64}) = likelihood(y, x, radar)

# Set up initial state particles
x0s = Vector{Vector{Float64}}()
dx = 10
dz = 10
dα = deg2rad(10)
dv = 3
for i in 1:Ns
    x = Vector{Float64}(undef, 4)
    x[1] = rand(Uniform(x0[1]-dx, x0[1]+dx)) # x
    x[2] = rand(Uniform(x0[2]-dz, x0[2]+dz)) # z
    x[3] = rand(Uniform(x0[3]-dα, x0[3]+dα)) # α
    x[4] = rand(Uniform(x0[4]-dv, x0[4]+dv)) # v
    push!(x0s, x)
end


pfMeanList = Vector{Vector{Float64}}()

kMax = 10
for k in params.ks[1:kMax]
    # Particle filter
    xPartList = SIR(x0s, Ns, y_list[1:k], params, dynamUp, pygx)

    # Get point estimate
    xMean = MMSE(xPartList)
    push!(pfMeanList, xMean)

end


xPfMat = stack(pfMeanList, dims=1)

plot(title="Particle Filter Test", xlabel="x", ylabel= "z")
scatter(xMat[1:kMax,1], xMat[1:kMax,2], label = "Exact Postion")
display(scatter!(xPfMat[1:kMax,1], xPfMat[1:kMax,2], label = "PF"))





