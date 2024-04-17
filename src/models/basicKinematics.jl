module basicKinematics

"""
This file hold basic kinematic models.
Manily used for estimators
"""

function simulate(xk::Vector{Float64}, dt::Float64)
    """
    Simulates the dynamics simulate the dynamics one k forward in time using numeric integration. 
    Uses simple linear dynamics
    Currently assuming no drag and no angle change. Should get to the rough right spot
    TODO: include noise? this would be nice becuase it would capture control inputs. Maybe a new function?
    
    Inputs:
        xk: state vector. [x, z, α, v]
        u: Vector of control inputs to inact for the full time step
        params: Problem parameters
    Outputs:
        xkp1: New x state at next time step
    """

    # Unpacking
    x = xk[1]
    z = xk[2]
    α = xk[3]
    v = xk[4]

    xd = v*cos(α)
    zd = v*sin(α)
    αd = 0.0
    vd = 0.0

    # Becuase alpha and v are constant, this can be spoofed to linearised dynamics:
    xkp1 = Vector{Float64}(undef, 4)
    xkp1[1] = x + xd * dt
    xkp1[2] = z + zd * dt
    xkp1[3] = α + αd * dt
    xkp1[4] = v + vd * dt

    return xkp1
end



end
