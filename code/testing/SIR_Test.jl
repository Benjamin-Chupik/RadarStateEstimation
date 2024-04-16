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
# SIR Testing
#-------------------------------------------- 
Ns = 500 # Number of prticles to use
SIR(x0, Ns, y_list, params, dynamUp, pygx)




