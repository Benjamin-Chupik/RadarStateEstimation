module fixedWing
using DifferentialEquations
# Infor on DIffiQ package: https://docs.sciml.ai/DiffEqDocs/stable/basics/overview/
using Distributions

using RadarStateEstimation.problemStruct
using RadarStateEstimation.models.radar

#include("../models/radarMeasurement.jl")

function fixedWingEOM(dx_vec, x_vec, p_vec, t)
    # x_vec: [x, y, α, v, w]
    # p_vec: Parameters vector: [uα, uv, Cd, ρ]

    # Unpacking
    x = x_vec[1]
    y = x_vec[2]
    α = x_vec[3]
    v = x_vec[4]
    w = x_vec[5]

    uα = p_vec[1]
    uv = p_vec[2]
    Cd = p_vec[3]
    ρ = p_vec[4] # dencity

    # Dyncamics Propogation
    dx_vec[1] = v*cos(α)
    dx_vec[2] = v*sin(α)
    dx_vec[3] = 0
    dx_vec[4] = -0.5* ρ * v^2*Cd
    dx_vec[5] = 0.0

    # Controls portion
    dx_vec[1] += 0
    dx_vec[2] += 0
    dx_vec[3] += uα
    dx_vec[4] += uv
    dx_vec[5] += 0.0

    # Dynamics Noise portion TODO

end

function fixedWingLinDyn(xk::Vector{Float64}, params::Params)
    α  = xk[3]
    v = xk[4]
    A = [0 0 -v*sin(α) cos(α) 0
         0 0  v*cos(α) sin(α) 0
         0 0 0 0 1
         0 0 0 -v*params.Cd/params.ρ
         0 0 0 0 0]
    return A
end

function fixedWingMeasDer(xk::Vector{Float64}, radar::Radar)

    xr, zr = radar.p

    x = xk[1]
    z = xk[2]
    α = xk[3]
    v = xk[4]
    e = atan(z-zr, x-xr)


    drdx = (x-xr)/sqrt((x-xr)^2+(z-zr)^2)
    drdz = (z-zr)/sqrt((x-xr)^2+(z-zr)^2)

    dedx = -(z-zr)/((x-xr)^2+(z-zr)^2)
    dedz = (x-xr)/((x-xr)^2+(z-zr)^2)

    drddv = cos(α-e)
    drddα = -v*sin(α-e)
    drddx = -v*sin(α-e)*-dedx
    drddz = -v*sin(α-e)*-dedz

    H = zeros(3,length(xk))

    H[1,1] = dedx
    H[1,2] = dedz

    H[2,1] = drdx
    H[2,2] = drdz

    H[3,1] = drddx
    H[3,2] = drddz
    H[3,3] = drddα
    H[3,4] = drddv

    return H
end

function simulate(xk::Vector{Float64}, u::Vector{Float64}, params::Params)
    """
    Simulates the dynamics simulate the dynamics one k forward in time using numeric integration. 
    Inputs:
        xk: state vector.
        u: Vector of control inputs to inact for the full time step
        params: Problem parameters
    Outputs:
        xkp1: New x state at next time step
    """

    # Setup ODE
    tspan = (0.0, params.dt) # dosent matter what time it starts on , just integrate for the dt time
    prob = ODEProblem(fixedWingEOM, xk, tspan, [u[1], u[2], params.Cd, params.ρ])
    
    xkp1 = solve(prob, Tsit5())[end] # only the last time step (if you dont do this it is a solution object that is weird)

    return xkp1
end


function genTrajectory(x0::Vector{Float64}, params::Params)
    """
    Updates the x_list to simulate the dynamics one k forward in time using numeric integration. 
    Inputs:
        x0: Initial state [x, z, α, v, w]
        Params: the problem parameters
    Outputs
        x_list: A list of states for every time step. 

    """

    # Crete the output list that is updated by simulate!
    x_list = Vector{Vector{Float64}}()
    push!(x_list, x0)

    u_list = Vector{Vector{Float64}}()

    # Go thorugh all the descrete times
    for k in params.ks

        # Generate the us TODO
        xk = x_list[end]
        
        u = [rand(Normal(0, deg2rad(2))), rand(Normal(2, 5))] # [α, V]
        push!(u_list, u)

        # Simulate one time step

        xkp1 = simulate(xk, u, params)
        push!(x_list, xkp1)
    end

    return x_list::Vector{Vector{Float64}}, u_list::Vector{Vector{Float64}}
end


end