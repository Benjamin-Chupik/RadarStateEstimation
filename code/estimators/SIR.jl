function SIR(x0, Ns, y_list, params, dynamUp, pygx)
    """
    Runs the Sample Importance Resample Particle Filter
    Inputs:
        x0: List of sampled initial state vectors
        Ns: Number of particles to use each time
        y_list: List of y measurement vectors
        params: Problem Parameters
        dynamUp: Dynamics update function. Takes in (x, dt) outputs x (should include noise)
        pygx: Function to return the probablity of yk gven xk (y, x) -> float (should include noise)
    """


    x_k = deepcopy(x0)
    x_kp1 = zeros(Ns) # Preallocate
    w_kp1 = zeros(Ns) # Preallocate
    # Go through every measurement
    for y in y_list[2:end]
        # For every particle, do SIS
        for i in 1:Ns
            # Sample new x_k
            x_kp1[i] = dynamUp(x_k[i], params.dt) # Popagate the dynamics

            # Get weightst
            w_kp1[i] = pygx(y, x_kp1[i]) # Get the measurement likelihood
        end

        # normalize weights (Still part of SIS)
        w_kp1 = w_kp1./sum(w_kp1)
        
        # SIR resampling
        if y != y_list[end] # Dont resample on the last one so weights make sence
            # Resample
            x_kp1 = sample(x_kp1, Weights(w_kp1), Ns)
        end

        # Move Data for next k and measurement
        x_k .= x_kp1
    end

    return x_k # returns the list of particles (They all have the same weight)
end
