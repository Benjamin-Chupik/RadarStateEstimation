include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasurement.jl")


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
# Plotting
#--------------------------------------------  

using Plots
# Dynamics
p = plot(title = "Dynamics testing")
xnames = ["x", "z", "α", "V"]
pList = []
for i in 1:4
    p_temp = scatter(params.ks, xMat[:,i], xlabel = "k", ylabel = xnames[i])
    push!(pList, p_temp)
end
display(plot(pList[1], pList[2], pList[3], pList[4], layout = (length(pList), 1) , title = "Dynamics testing", size=(400,800)))


# Measuremetns
ynames = ["elevation", "Range", "Range Velocity"]
pList = []
for i in 1:3
    p_temp = scatter(params.ks, yMat[:,i], xlabel = "k", ylabel = ynames[i])
    plot!(params.ks, yMat_noNoise[:,i])
    push!(pList, p_temp)
end
display(plot(pList[1], pList[2], pList[3], layout = (length(pList), 1), title = "Measurement testing", size=(400,800)))


x_fromy = y2p(y_list, radar)
x_fromy_mat = stack(x_fromy, dims=1)
p = plot(xMat[:,1], xMat[:,2], label = "Exact Position")
scatter!(x_fromy_mat[:,1], x_fromy_mat[:,2], label = "Position from measurements")
display(p)


# Testing random
i = 10
likelihood(y_list[i], x_list[i], radar)