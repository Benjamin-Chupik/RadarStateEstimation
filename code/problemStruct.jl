struct Params
    dt::Float64  # time step
    maxT::Float64  # Max time
    ts::Vector{Float64}
    ks::Vector{Int}
    Cd::Float64
    ρ::Float64

    # Constructor
    function Params(dt::Float64, maxT::Float64, Cd::Float64, ρ::Float64)
        t = collect(1:dt:maxT)
        k = collect(1:length(t))
        new(dt, maxT, t, k, Cd, ρ)
    end
end