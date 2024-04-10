
function EKF_step(xk, yk1, uk, Pk, Qk, Rk)
    # n = length(xk)

    ############### PREDICTION STEP ################
    xk1min = f(xk, uk, dt) # nonlinear dynamics prediction
    Atild = genA(xk1min) # linearized dynamics at timestep
    Fk = I + dt*Atild 

    Pk1min = Fk*Pk*Fk' + Qk # update Pmin

    yk1hat = h(xk1min)
    Hk1 = genH(xk1min)
    ek1 = yk1 .- yk1hat
    Kk1 = Pk1min*Hk1'*inv(Hk1*Pk1min*Hk1'+ Rk)

    xk1plus = xk1min + Kk1*ek1
    Pk1plus = (I-Kk1*Hk1)*Pk1min
    return (xk1plus, Pk1plus)
end

# STILL NEED df, genA, 
function genA(xk)
    alpha = xk[3]
    v = xk[4]
    A = [0 0 -v*sin(alpha) cos(alpha)
         0 0  v*cos(alpha) sin(alpha)
         0 0 0 0
         0 0 0 -2*v*Cd/rho]
    return A
end

