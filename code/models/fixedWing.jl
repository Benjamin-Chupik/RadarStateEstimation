using DifferentialEquations
# Infor on DIffiQ package: https://docs.sciml.ai/DiffEqDocs/stable/basics/overview/

struct Params
    dt::Float64  # time step
    maxT::Float64  # Max time
    ts::Vector{Float64}
    ks::Vector{Int}
    Cd::Float64

    # Constructor
    function Params(dt::Float64, maxT::Float64, Cd::Float64)
        t = collect(1:dt:maxT)
        k = collect(1:length(t))
        new(dt, maxT, t, k, Cd)
    end
end



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


function simulate!(x_list::Vector{Vector{Float64}}, u::Vector{Float64}, params::Params)
    """
    Updates the x_list to simulate the dynamics one k forward in time using numeric integration. 
    Inputs:
        x_list: List of state vectors. Needs to be a list (not matrix) so you an push to it. Also needs at least one entry
        u: Vector of control inputs to inact for the full time step
        params: Problem parameters
    Outputs:
        None, it updates x_list with a new state
    """

    # Setup ODE
    x0 = x_list[end]
    tspan = (0.0, params.dt) # dosent matter what time it starts on , just integrate for the dt time
    prob = ODEProblem(fixedWingEOM, x0, tspan, [u[1], u[2], params.Cd])

    t, x = solve(prob, Tsit5())

    push!(x_list, x[end]) # only record at the dt time

    return nothing
end


function simulate(x0::Vector{Float64}, params::Params)
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
        simulate!(x_list, u, prams)
    end

    return x_list::Vector{Vector{Float64}}
end


if false
    # TEstning
    Cd = 1.0
    prams = Params(.5, 10.0, Cd)
    x0 = [0.0, 0.0, 0.0, 5.0]

    x_list = simulate(x0, prams)

    using Plots
    p = plot()
    for x in x_list
        scatter!([x[1]], [x[2]])
    end

    display(p)
end
