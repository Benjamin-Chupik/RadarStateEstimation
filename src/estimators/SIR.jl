module SIR

using StatsBase
using RadarStateEstimation.problemStruct
using Distributions: Normal, Uniform
using DifferentialEquations: ODEProblem, solve, Tsit5

function SIR_update(x0::Vector{Vector{Float64}}, wk::Vector{Float64}, ys::Vector{Vector{Float64}}, params::Params, dynamUp::Function, pygx::Function)
    """
    Note: Gives warnings, not sure if it works? :/
    Updates the particls list xk to propogate dynamcs, and take the measurements into account
    Inputs
        xk: particles
        wk: Particle weights NOTE: should allways be 1 for SIR
        ys: list of measurement vector received at k+1 (ys[1] should correspond to the next k, not the k for the inputed x0)
        params: Problem Parameters (Struct)
        dynamUp: Dynamics update function. Takes in (x, dt) outputs x (should include noise)
        pygx: Function to return the probablity of yk gven xk (y, x) -> float (should include noise)
    Outputs:
        xk: the xs for the final measurement. 
    """

    xk = deepcopy(x0)

    for y in ys
        SIR_update!(xk, wk, y, params, dynamUp, pygx)
    end

    return xk
end


function SIR_update!(xk::Vector{Vector{Float64}}, wk::Vector{Float64}, y::Vector{Float64}, params::Params, dynamUp::Function, pygx::Function)
    """
    Updates the particls list xk to propogate dynamcs, and take the measurement into account
    Inputs
        xk: particles (updated)
        wk: Particle weights (updated) NOTE: should allways be 1 for SIR
        y: measurement vector received at k+1
        params: Problem Parameters (Struct)
        dynamUp: Dynamics update function. Takes in (x, dt) outputs x (should include noise)
        pygx: Function to return the probablity of yk gven xk (y, x) -> float (should include noise)
    """

    Ns = length(wk)

    for i in 1:Ns
        xk[i] .= dynamUp(xk[i], params.dt) # Popagate the dynamics
        wk[i] = pygx(y, xk[i]) # Get the measurement likelihood
    end

    # normalize weights (Still part of SIS)
    wk .= wk./sum(wk)

    # Resample
    xk .= sample(xk, Weights(wk) , Ns)

    return nothing
end


############################################
# Point Estimators from Particles
############################################

function MMSE(xParticleList::Vector{Vector{Float64}})
    """
    Returns the MMSE point estimate for a particle filter (Assuming equal weights)
    """

    xMat = stack(xParticleList, dims=1)

    return dropdims(mean(xMat, dims=1), dims=1)::Vector{Float64}, dropdims(std(xMat, dims=1), dims=1)::Vector{Float64}

end

############################################
# Models for motion
############################################

function dynamicsUpdate(xk, params::Params)
    """
    Update the dynamics of a particle. 
    NOTE: ParticleFilters.jl is similar format ish.
    Inputs:
        x: the state vector
        dt: change in time
    """

    # Dynamics
    function fixedWingEOM(dx_vec, x_vec, p_vec, t)
        # x_vec: [x, y, α, v, w]
        # p_vec: Parameters vector: [noise (vector), Cd, rho]
    
        # Unpacking
        x = x_vec[1]
        y = x_vec[2]
        α = x_vec[3]
        v = x_vec[4]
        w = x_vec[5]
    
        noise = p_vec[1]
        Cd = p_vec[2]
        rho = p_vec[3]

        # Dyncamics Propogation
        dx_vec[1] = v*cos(α)
        dx_vec[2] = v*sin(α)
        dx_vec[3] = w
        dx_vec[4] = -0.5*v^2*Cd*rho
        dx_vec[5] = 0 # no rotational drag

        # NOise
        dx_vec[1] += noise[1]
        dx_vec[2] += noise[2]
        dx_vec[3] += noise[3]
        dx_vec[4] += noise[4]
        dx_vec[5] += noise[5]
    end

    # Noise
    w = [   Uniform(-.001, .001), # x_dot uncertianty (from wind)
            Uniform(-.001, .001), # z_dot uncertainty (from wind)
            Normal(deg2rad(0), deg2rad(2.0)), # alpha_dot uncertainty (from controls)
            Normal(25, 5), # v_dot uncertainty (from controls)
            0.0 # w_dot uncertainty (not used)
        ]
    
    # Generate random number:
    w_samp = rand.(w[1:end-1])
    push!(w_samp, 0.0)

    # Setup ODE 
    tspan = (0.0, params.dt) # dosent matter what time it starts on , just integrate for the dt time
    prob = ODEProblem(fixedWingEOM, xk, tspan, [ w_samp, params.Cd, params.ρ])
    
    # TODO: make sure that end is actually the time dt, if not need to poll at t=dt for correct position
    xkp1 = solve(prob, Tsit5())[end] # only the last time step (if you dont do this it is a solution object that is weird)

    return xkp1
end
end

