module RadarStateEstimation

export  problemStruct,
        models,
        estimators


include("problemStruct.jl") # Params struct
include("models/models.jl")
include("estimators/estimators.jl")

greet() = print("Hello World!")

end # module RadarStateEstimation
