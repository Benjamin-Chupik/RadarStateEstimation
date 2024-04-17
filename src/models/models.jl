module models
    
export  fixedWing,
        radar,
        dubinsAiraft,
        basicKinematics


# Note: For some reason you need to include radar before fixed wing becaues fixed wing uses radar... this could be problems
include("radar.jl")
include("fixedWing.jl")
include("dubinsAircraft.jl")
include("basicKinematics.jl")




end