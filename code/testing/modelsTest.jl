include("../problemStruct.jl")
include("../models/fixedWing.jl")
include("../models/radarMeasure.jl")



# Dynamics
Cd = 1.0
prams = Params(.5, 10.0, Cd)
x0 = [0.0, 0.0, 0.0, 5.0]
x_list = simulate(x0, prams)

# Measurements
#Radar([50,0], rNoise, rdNoise, azNoise, elNoise)
#y_list = radarMeasure(x_list, radar::Radar)


using Plots
p = plot(title = "Model testing")
for x in x_list
    scatter!([x[1]], [x[2]])
end

display(p)