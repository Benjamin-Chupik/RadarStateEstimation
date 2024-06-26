# Estimators

There are multiple ways to determine the state of something from measurements. The simplest viable solution is applied, and then more complex solutions are added onto it. Due to the non-linear model, the EKF is first used. Then a Bootstrap Particle Filter is applied to compare effectiveness. 


## EKF


## Particle Filter
The variant of the particle filter (PF) used for this project is the Bootstrap PF. This filter resamples particles every step, but can still have particle collapse if an unexpected measurement is received. 

The basics of the bootstrap particle filter is to generate $x_{k+1}^i$ using a generative dynamics model with noise, then to get weights of the particles. For the weights' equation, $q(x_{k+1}|x_k,y_{k+1})$ is set to $p(x_{k+1}|x_k)$ so that the weights' equation simplifies to: 
$$
w_{k+1}^i \propto p(y_k|x_k)
$$
Then the particles are resampled using the weights and the process moves 1 time step forward. 

This means a generative dynamics model and a likelihood function $\Lambda$ are needed for the PF (The likelihood function should depend on the dynamics model). These do not have to be the same as the dynamics because in the real world, you may not know the dynamics.

Different generative dynamics models are derived: 

### Generative Dynamics Model 

#### Basic Kinematics

#### Fixed Wing
The fixed wing generative dynamics model is the same as the one derived in [modeling](./modeling.md), but it has different noise, and no control vector. This is because we are assuming that the estimator does not know the control inputs of the aircraft. Because of this, the control input uncertainty needs to be included in the model noise. As a reminder, wind noise is affecting the positional dynamics, and control is affecting the rate dynamics.

The wind noise is generated from a ____________, which is not intuitive to implement here. Instead, the upper and lower bounds are used to create a uniform distribution so that the particles will be generated in all possible spots. 

The control noise is easier because we know how the controls are generated. These distributions are used, but scenarios where we don't know this noise can be looked at by replacing these distributions. We also know that there is no $w$ dynamics noise, so it is set to zero, but can be changed to look at the affect of estimator errors.

$$
u = 0
$$

$$
w = 
\begin{bmatrix}
\mathcal{U}[-2,2] \\
\mathcal{U}[-2,2] \\
\mathcal{N}(0, 3.0) \text{deg} \\
\mathcal{N}(1, 3.0) \text{m/s} \\
0.0
\end{bmatrix}
$$

#### Multirotor
The multirotor generative dynamics model is the same as the one derived in [modeling](./modeling.md), but it has different noise, and no control vector. This is because we are assuming that the estimator does not know the control inputs of the aircraft. Because of this, the control input uncertainty needs to be included in the model noise. 

Becuasae the multirotor is the same model as the fixed wind except for the control noise, the above noise needs to only be slightly modified for the controls.

$$
u = 0
$$

$$
w = 
\begin{bmatrix}
\mathcal{U}[-2,2] \\
\mathcal{U}[-2,2] \\
0.98\mathcal{N}(0, 3.0) +  0.01\mathcal{N}(90, 1.0) + 0.01\mathcal{N}(-90, 1.0) \hspace{1cm} \text{[deg/s]} \\
\mathcal{N}(1, 3.0) \hspace{1cm} \text{[m/s/s]} \\
0.0
\end{bmatrix}
$$


### Likelihood Function

#### Radar Measurement Likelihood
The measurement likelihood function is derived using the radar model from [modeling](./modeling.md). The measurements are conditionally independent of each other given x, so
$$
p(y_k|x_k) = p(y1_k|x_k)*p(y2_k|x_k)*p(y3_k|x_k)
$$
Where, $y1_k$ is the elevation measurement at time k, $y2$ is the range, and $y3$ is the range velocity. 
$$
p(y1_k|x_k) = \mathcal{N}(\tan^{-1}{\frac{z-z_r}{x-x_r}}, 2)
$$
$$
p(y2_k|x_k) = \mathcal{X}^2( 2) + \sqrt{(x-x_r)^2+(z-z_r)^2}
$$
$$
p(y3_k|x_k) = \mathcal{N}(v \cos{\alpha - \tan^{-1}{\frac{z-z_r}{x-x_r}}}, 0.2)
$$
