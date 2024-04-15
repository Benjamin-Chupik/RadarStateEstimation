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
function rNoise()
    return rand(Chisq(4))
end
function rdNoise()
    #return 0.0
    return rand(Normal(0,.2))
end
function azNoise()
    #return 0.0
    return rand(Normal(0.0, deg2rad(2)))
end
function elNoise()
    #return 0.0
    return rand(Normal(0.0, deg2rad(2)))
end
radar = Radar([50.0,0.0], rNoise, rdNoise, azNoise, elNoise)
y_list = radarMeasure(x_list, radar)

xMat = stack(x_list, dims=1)
yMat = stack(y_list, dims=1)


#--------------------------------------------  
# Plotting
#--------------------------------------------  

using Plots
# Dynamics
p = plot(title = "Dynamics testing")
scatter!(xMat[:,1], xMat[:,2])
display(p)

# Measuremetns
ynames = ["elevation", "Range", "Range Velocity"]
pList = []
for i in 1:3
    p_temp = scatter(params.ks, yMat[:,i], xlabel = "k", ylabel = ynames[i])
    push!(pList, p_temp)
end

@show pList
display(plot(pList[1], pList[2], pList[3], layout = (length(pList), 1))) #, title = "Measurement testing"))

x_fromy = y2p(y_list, radar)
x_fromy_mat = stack(x_fromy, dims=1)
p = plot(xMat[:,1], xMat[:,2], label = "Exact Position")
scatter!(x_fromy_mat[:,1], x_fromy_mat[:,2], label = "Position from measurements")
display(p)