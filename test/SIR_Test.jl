using RadarStateEstimation
using RadarStateEstimation.problemStruct # This exports Parmas

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
# SIR Testing
#-------------------------------------------- 
# Other setup
Ns = 1000 # Number of prticles to use
dynamUp(x::Vector{Float64}, u::Vector{Float64}, dt::Float64) = RadarStateEstimation.models.fixedWing.simulate(x, u, Params(dt, 100.0, Cd, .2))
pygx(y::Vector{Float64}, x::Vector{Float64}) = RadarStateEstimation.models.radar.likelihood(y, x, radar)

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
pfSTDList = Vector{Vector{Float64}}()

kMax = 10
for k in params.ks[1:kMax]
    # Particle filter
    xPartList = RadarStateEstimation.estimators.SIR.SIR_update(x0s, u_list, Ns, y_list[1:k], params, dynamUp, pygx)

    # Get point estimate
    xMean, xSTD = RadarStateEstimation.estimators.SIR.MMSE(xPartList)
    push!(pfMeanList, xMean)
    push!(pfSTDList, xSTD)

end


xPfMat = stack(pfMeanList, dims=1)
xPfSTDMat = stack(pfSTDList, dims=1)

plot(title="Particle Filter Test", xlabel="x", ylabel= "z")
scatter(xMat[1:kMax,1], xMat[1:kMax,2], label = "Exact Postion")
display(scatter!(xPfMat[1:kMax,1], xPfMat[1:kMax,2], label = "PF"))


p = plot(title = "Dynamics testing")
xnames = ["x", "z", "α", "V"]
pList = []
for i in 1:4
    p_temp = scatter(params.ks, xMat[:,i], xlabel = "k", ylabel = xnames[i])
    scatter!(params.ks[1:kMax], xPfMat[1:kMax,i], yerr=xPfSTDMat[1:kMax, i], label = "PF")
    push!(pList, p_temp)
end
display(plot(pList[1], pList[2], pList[3], pList[4], layout = (length(pList), 1) , title = "Dynamics testing", size=(400,800)))






