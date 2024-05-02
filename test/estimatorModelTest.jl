using RadarStateEstimation
using RadarStateEstimation.problemStruct # This exports Parmas
using RadarStateEstimation.estimators.SIR
import RadarStateEstimation.models.fixedWing as physMod
import RadarStateEstimation.models.radar as radMod

using Distributions
using Plots


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

#xMat = stack(x_list, dims=1)
#yMat = stack(y_list, dims=1)
#yMat_noNoise = stack(y_list_noNoise, dims=1)
y_pos = radMod.y2p(y_list, radar)


#--------------------------------------------  
# Dynamics  Testing for new function
#--------------------------------------------  


idLook = 1
x_look = x_list[idLook] # pull the 
np = 1000
x_pred = [] # list of particle states
x_pred_like = [] # list of particle likelihoods
p = plot(title = "Dynamics test", legend = false)

x_look = vcat(x_look, w_list[idLook])

for i = 1:np
    predX =  SIR.dynamicsUpdate_NEW(x_look, params)
    predXLike = radMod.likelihood(y_list[idLook+1], predX, radar)
    push!(x_pred,predX)
    push!(x_pred_like,predXLike)
    if predXLike==0
        c=:gray
    else
        c=:blue
    end
    scatter!(p, [predX[1]], [predX[2]], color=c)
end
x_pred_like .= x_pred_like./sum(x_pred_like) # normalize like in PF

x_pred_mat = stack(x_pred, dims=1)
scatter!(p, [x_list[idLook][1]], [x_list[idLook][2]], color=:red)
scatter!(p, [x_list[idLook+1][1]], [x_list[idLook+1][2]], color=:red)
scatter!(p, [x_list[idLook+1][1]], [x_list[idLook+1][2]], color=:red)

#p = plot(x_pred_mat[:,1], x_pred_mat[:,2], x_pred_like, st=:scatter) #camera=(45,0)
display(p)


display(plot(x_pred_like))