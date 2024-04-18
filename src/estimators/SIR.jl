module SIR

using StatsBase
using RadarStateEstimation.problemStruct

function SIR_update(x0::Vector{Vector{Float64}}, u_list::Vector{Vector{Float64}}, Ns::Int64, y_list::Vector{Vector{Float64}}, params::Params, dynamUp::Function, pygx::Function)
    """
    Runs the Sample Importance Resample Particle Filter
    Inputs:
        x0: List of sampled initial state vectors. Needs to be of size Ns of states
        Ns: Number of particles to use each time
        y_list: List of y measurement vectors
        params: Problem Parameters
        dynamUp: Dynamics update function. Takes in (x, dt) outputs x (should include noise)
        pygx: Function to return the probablity of yk gven xk (y, x) -> float (should include noise)
    """

    X = Vector{Vector{Vector{Float64}}}()
    push!(X, deepcopy(x0))
    x_kp1 = deepcopy(x0)
    
    w_kp1 = Vector{Float64}(undef, Ns) # Preallocate Weights for next particles
    # Go through every measurement
    for y_id in 2:length(y_list) # start at 2nd measuremnt becuase asssume first is for x0
        y = y_list[y_id]
        u = u_list[y_id-1]
        # For every particle, do SIS
        for i in 1:Ns
            x_kp1[i] = dynamUp(X[end][i], u, params.dt) # Popagate the dynamics
            w_kp1[i] = pygx(y, x_kp1[i]) # Get the measurement likelihood
        end

        # normalize weights (Still part of SIS)
        w_kp1 = w_kp1./sum(w_kp1)
        
        # SIR resampling
        if y != y_list[end] # Dont resample on the last one so weights make sence
            # Resample
            w_temp = Weights(w_kp1)
            x_kp1 = sample(x_kp1, w_temp , Ns)
        end
        push!(X, deepcopy(x_kp1))
        # Move Data for next k and measurement
        # x_k .= x_kp1
    end

    return X # returns the list of particles (They all have the same weight)
end


function MMSE(xParticleList::Vector{Vector{Float64}})
    """
    Returns the MMSE point estimate for a particle filter (Assuming equal weights)
    """

    xMat = stack(xParticleList, dims=1)

    return dropdims(mean(xMat, dims=1), dims=1)::Vector{Float64}, dropdims(std(xMat, dims=1), dims=1)::Vector{Float64}

end

end