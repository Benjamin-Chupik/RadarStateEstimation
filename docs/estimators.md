# Estimators

There are multiple ways to determine the state of something from measurements. The simplest viable solution is applied, and then more complex solutions are added onto it. Due to the non-linear model, the EKF is first used. Then a Bootstrap Particle Filter is applied to compare effectiveness. 


## EKF


## Particle Filter
The variant of the particle filter (PF) used for this project is the Bootstrap PF. This filter resamples particles every step, but can still have particle collapse if an unexpected measurement is received. A generative dynamics model and a likelihood function $\Lambda$ are needed for the PF (The likelihood function should depend on the dynamics model). These do not have to be the same as the dynamics because in the real world, you may not know the dynamics.

### Generative Dynamics Model 

#### Basic Kinematics

#### Fixed Wing

#### Multirotor

### Likelihood Function


