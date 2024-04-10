using LinearAlgebra

dt = 0.05
function genH(xk)
    alpha = xk[3]
    v = xk[4]
    H = [0 0 -v*sin(alpha) cos(alpha)
         0 0  v*cos(alpha) sin(alpha)
         0 0 0 0
         0 0 0 -2*v*Cd/rho]
    return I+dt*H
end

X = [300, 300, 0.1, 50]
Cd = 0.05
rho = 1

println(genH(X)*X)
println(X + dt*[X[4]*cos(X[3]), X[4]*sin(X[3]), 0, -X[4]^2/rho*Cd])