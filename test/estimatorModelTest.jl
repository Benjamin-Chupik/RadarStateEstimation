using RadarStateEstimation
using RadarStateEstimation.problemStruct # This exports Parmas
using RadarStateEstimation.estimators.SIR
import RadarStateEstimation.models.fixedWing as physMod
import RadarStateEstimation.models.radar as radMod

using Distributions
using Plots


#--------------------------------------------  
# Dynamics  Generation
#--------------------------------------------  
# Dynamics
Cd = .1
params = Params(.5, 100.0, Cd, 1.0)
x0 = [0.0, 50.0, 0.0, 25.0, 0.0] # [x, z, alpha, v, w]
x_list, u_list, w_list = physMod.genTrajectory(x0, params)



#--------------------------------------------  
# Dynamics  Testing for new function
#--------------------------------------------  
idLook = 10
x_look = x_list[idLook] # pull the 
np = 1000
x_part = []
p = plot(title = "Dynamics test", legend = false)

x_look = vcat(x_look, w_list[idLook])

for i = 1:np
    predX = SIR.dynamicsUpdate_NEW(x_look, params)
    push!(x_part,predX)
    scatter!(p, [predX[1]], [predX[2]], color = :blue)
end
scatter!(p, [x_look[1]], [x_look[2]], color=:red)
scatter!(p, [x_list[idLook+1][1]], [x_list[idLook+1][2]], color=:red)
display(p)
