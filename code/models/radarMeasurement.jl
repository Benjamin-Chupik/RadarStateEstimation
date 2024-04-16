# Seems needlessly complex to use now, but might be a good idea later if we try to introduce how much surface area the object has shown or something...
# struct State(t)
#     #A struct that holds all the info for the state of a object at one time

#     t::Float64
#     x
#     y
#     z
#     # orientation

# end
using LinearAlgebra
using Printf
using Distributions

#--------------------------------------------  
# Data Structures
#--------------------------------------------  

struct Radar
    """
    A struct that holds all the info of a radar setup. 
    """

    # Positioning
    p::Vector{Float64} # location of radar ([x,z])
    #@assert length(p) == 2 "Input vector must have a length of 6"
    # a::Vector{Float64} # Attitude of radar (wherre it is pointing)
    # @assert length(a) == 3 "Input vector must have a length of 6"

    #Things we can add: max, range, min, range, FOV bounds, ...

    # Noises
    rNoise::UnivariateDistribution # returns a sample for the range noise 
    rdNoise::UnivariateDistribution # returns a sample for the range velocity noise
    elNoise::UnivariateDistribution # returns a sample for the range noise
end


#--------------------------------------------  
# Measurement Generation
#--------------------------------------------  

function radarMeasure(x::Vector{Float64}, radar::Radar)
    """
    Simulates a radar measuring a object flying through space. Only returns the measured center of mass of the object. 

    Inputs:
        x: the state of the aircraft. [x, y, α, v]
    Outputs:
        y: the measurement of the aircraft. [el, r, rd]
            el: elevation
            r: range
            rd: range velocity
    """

    dp = x[1:2] - radar.p # get the delta position [x,y,z]
    α = x[3]
    v = x[4]
    r = norm(dp) + rand(radar.rNoise)
    v_vector = [cos(α), sin(α)].*v
    rd = dot(dp, v_vector)/norm(dp) + rand(radar.rdNoise)# Get the component of the velocity pointing in the direction of the radar (same direction as dp) 
    #rd_math = v*cos(α - atan(x[2]-radar.p[2], x[1]-radar.p[1]))
    #@printf("%f\n\tVector: %f\n\tTrig: %f\n", rd-rd_math, rd, rd_math )
    el = atan(dp[2], dp[1]) + rand(radar.elNoise)

    y = [el, r, rd]

    return y::Vector{Float64}
end


function radarMeasure(xs::Vector{Vector{Float64}}, radar::Radar)
    """
    Simulates a radar measuring a object flying through space. Only returns the measured center of mass of the object. 

    Inputs:
        xs: the vector of states of the aircraft through discrete time. [x, y, α, v]
    Outputs:
        y: the measurements of the aircraft through discrete time . [el, r, rd]
            el: elevation
            r: range
            rd: range velocity
    """
    ys = Vector{Vector{Float64}}()

    # Loop through every time
    for x in xs

        y = radarMeasure(x, radar)
        push!(ys, y)
    end

    return ys::Vector{Vector{Float64}}
end

function radarMeasure_noNoise(x::Vector{Float64}, radar::Radar)
    """
    Simulates a radar measuring a object flying through space. Only returns the measured center of mass of the object. 

    Inputs:
        x: the state of the aircraft. [x, y, α, v]
    Outputs:
        y: the measurement of the aircraft. [el, r, rd]
            el: elevation
            r: range
            rd: range velocity
    """

    dp = x[1:2] - radar.p # get the delta position [x,y,z]
    α = x[3]
    v = x[4]
    r = norm(dp)
    v_vector = [cos(α), sin(α)].*v
    rd = dot(dp, v_vector)/norm(dp) # Get the component of the velocity pointing in the direction of the radar (same direction as dp) 
    #rd_math = v*cos(α - atan(x[2]-radar.p[2], x[1]-radar.p[1]))
    #@printf("%f\n\tVector: %f\n\tTrig: %f\n", rd-rd_math, rd, rd_math )
    el = atan(dp[2], dp[1])

    y = [el, r, rd]

    return y::Vector{Float64}
end

function radarMeasure_noNoise(xs::Vector{Vector{Float64}}, radar::Radar)
    """
    Simulates a radar measuring a object flying through space. Only returns the measured center of mass of the object. 

    Inputs:
        xs: the vector of states of the aircraft through discrete time. [x, y, α, v]
    Outputs:
        y: the measurements of the aircraft through discrete time . [el, r, rd]
            el: elevation
            r: range
            rd: range velocity
    """
    ys = Vector{Vector{Float64}}()

    # Loop through every time
    for x in xs

        y = radarMeasure_noNoise(x, radar)
        push!(ys, y)
    end

    return ys::Vector{Vector{Float64}}
end


#--------------------------------------------  
# likelihood function
#--------------------------------------------  

function likelihood(y::Vector{Float64}, x::Vector{Float64}, radar::Radar)
    """
    A general likelihood function for the radar model.
    Becuase the measurements are independent:
    P(y|x) = P(y1|x)*P(y2|x)*P(y3|x)
    The P(y1|x) is from the noise model associated with the radar input
    Inputs:
        y: the measurement vector [el, r, rd]
        x: the state vector [x, y, α, v]
    Outputs:
        likelihood: The P(y|x) 
    """

    # Y predicted with no noise
    y_measure = radarMeasure_noNoise(x, radar)

    Λ = Vector{Float64}(undef, 3)
    Λ[1] = pdf(y_measure[1]+radar.elNoise, y[1]) # el
    Λ[2] = pdf(y_measure[2]+radar.rNoise, y[2]) # r
    Λ[3] = pdf(y_measure[3]+radar.rdNoise, y[3]) # rdot
    return prod(Λ)

end


#--------------------------------------------  
# Usefull Helper Functions
#--------------------------------------------  

function y2p(y::Vector{Float64}, radar::Radar)
    """
    Converts a y measurement vector to a x state vector
    inputs:
        y: [el, r, rd]
    outputs
        p: [x, y]
    """

    el = y[1]
    r = y[2]
    rd = y[3]


    dP = [cos(el), sin(el)].*r

    p = dP + radar.p

    return p
end

function y2p(ys::Vector{Vector{Float64}}, radar::Radar)
    ps = []
    for y in ys
        push!(ps, y2p(y, radar))
    end
    return ps
end