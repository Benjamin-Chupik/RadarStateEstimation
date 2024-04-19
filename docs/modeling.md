This project relies on generative dynamics models to give paths for estimation, and estimate trajectories. The models can include any type of noise, be deterministic, or even have discrete switches in dynamics. We embraced the saying 'All models are wrong, but some models are useful' and focused on the resulting behavior of the models rather than coding up realistic physics and aerodynamics. The models in this section are used for both generating paths and inside the estimators. 

# Generative Dynamics models.
There are 2 main dynamics models, the object models which include physical objects moving in space, and the measurement models which give measurement behavior for tracking objects. 

## Object Models
### Fixed Wing 
The goal of the fixed wing model is to generate a smooth trajectory that has maneuvers in it. It should look like a plausible trajectory for a fixed wing aircraft. This means no sudden turns, the velocity should always stay above some value $v_{min}$, and it should look like it is gliding. Most importantly, it should have some random noise included into the model to make the resulting trajectory interesting. This can be accomplished many ways, some of which are described bellow. 

#### Dubens light
This method takes inspiration from dubens aircraft model, but makes some simplifications to make a more intuitive model. 

$$
x = 
\begin{bmatrix}
x \\
z \\
\alpha \\
v
\end{bmatrix}
=
\begin{bmatrix}
\text{x position} \\
\text{y position} \\
\text{Attitude angle from horizontal} \\
\text{Velocity}
\end{bmatrix}
$$

$$
\dot{x} = 
\begin{bmatrix}
v \cos{\alpha} \\
v \sin{\alpha} \\
0\\
-\frac{1}{2}v^2 Cd
\end{bmatrix}
+
\omega
+
u
$$
where $\omega$ is the noise vector and $u$ is the control vector.

$$
u = 
\begin{bmatrix}
0 \\
0 \\
d\alpha \\
dv
\end{bmatrix}
$$

This is a non-linear model that must be integrated using some numeric integrator. We use RK45 for this purpose. The control vector and noise vector are fixed for each $k$ to $k+1$ allowing for easyer integration and control of the aircraft. 

The control vector is generated randomly each time step from the following distribution

$$
u = 
\begin{bmatrix}
0 \\
0 \\
\mathcal{N}(0, 3.0) \text{deg} \\
\mathcal{N}(1, 3.0) \text{m/s}
\end{bmatrix}
$$

This generates the following trajectories:

![Fixed Wing Example Path](./figures/fixedWingDubensSimpleExampleTrajectory.svg)

This is a realistic looking aircraft trajectory with the pilot ascending up and then curving down. The specific parameters may be tunned to give slightly differenet behavior, but the general shapes of the trajectories generated will stay the same. 

#### Dubens

### Multirotor - Dubens
The goal of the Multirotor model is to generate a smooth trajectory that has general maneuvers in it, and some multirotor specific maneuvers. It should look like a plausible trajectory for a multirotor aircraft. This trajectory should look similar to the fixed wing model for most of the time, but should allow for occasional sharp turns, and lowering the velocity to zero.




## Measurement Models

### Radar
