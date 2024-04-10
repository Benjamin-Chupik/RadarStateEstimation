using DifferentialEquations
# Infor on DIffiQ package: https://docs.sciml.ai/DiffEqDocs/stable/basics/overview/

include("../problemStruct.jl")
#using. problemStruct

function fixedWingEOM(dx_vec, x_vec, p_vec, t)
    # x_vec: [x, y, α, v]
    # p_vec: Parameters vector: [uα, uv, Cd]

    # Unpacking
    x = x_vec[1]
    y = x_vec[2]
    α = x_vec[3]
    v = x_vec[4]

    uα = p_vec[1]
    uv = p_vec[2]
    Cd = p_vec[3]

    # Dyncamics Propogation
    dx_vec[1] = v*cos(α)
    dx_vec[2] = v*sin(α)
    dx_vec[3] = 0
    dx_vec[4] = -v^2*Cd

    # Controls portion
    dx_vec[1] += 0
    dx_vec[2] += 0
    dx_vec[3] += uα
    dx_vec[4] += uv

    # Dynamics Noise portion TODO

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
    prob = ODEProblem(fixedWingEOM, xk, tspan, [u[1], u[2], params.Cd])

    t, x = solve(prob, Tsit5())

    xkp1 = x[end] # only the last time step (if you dont do this it is a solution object that is weird)

    return xkp1
end


function genTrajectory(x0::Vector{Float64}, params::Params)
    """
    Updates the x_list to simulate the dynamics one k forward in time using numeric integration. 
    Inputs:
        x0: Initial state [x, z, α, v]
        Params: the problem parameters
    Outputs
        x_list: A list of states for every time step. 

    """

    # Crete the output list that is updated by simulate!
    x_list = Vector{Vector{Float64}}()
    push!(x_list, x0)
    
    # Go thorugh all the descrete times
    for k in params.ks

        # Generate the us TODO
        u = [0.0, 0.0]

        # Simulate one time step
        xkp1 = simulate(x_list[end], u, params)
        push!(x_list, xkp1)
    end

    return x_list::Vector{Vector{Float64}}
end
