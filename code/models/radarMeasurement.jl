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
    rNoise::Function # returns a sample for the range noise 
    rdNoise::Function # returns a sample for the range velocity noise
    azNoise::Function # returns a sample for the range noise
    elNoise::Function # returns a sample for the range noise
end


function radarMeasure(xs::Vector{Vector{Float64}}, radar::Radar)
    """
    Simulates a radar measuring a object flying through space. Only returns the measured center of mass of the object. 

    Inputs:
        xs: the state of the aircraft through discrete time. [x, y, α, v]
    Outputs:
        y: the measurements of the aircraft through discrete time . [el, r, rd]
            el: elevation
            r: range
            rd: range velocity
    """
    ys = []

    # Loop through every time
    for x in xs
        dp = x[1:2] - radar.p # get the delta position [x,y,z]
        α = x[3]
        v = x[4]
        r = norm(dp) + radar.rNoise()
        v_vector = [cos(α), sin(α)].*v
        rd = dot(dp, v_vector)/norm(dp) + radar.rdNoise()# Get the component of the velocity pointing in the direction of the radar (same direction as dp) 
        el = atan(dp[2], dp[1]) + radar.elNoise()

        push!(ys, [el, r, rd])
    end

    return ys
end