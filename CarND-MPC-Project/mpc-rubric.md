##1. Implementation##

###1.1 Model###
I have used a kinematic model of the car. The state of the model is captured by 6 variables:
- x, y represent the position
- psi the orientation
- v the velocity
- cte the cross track error, or the error distance relative to the reference trajectory
- psie, the orientation error with respect to the reference trajectory

The model also inludes two actuators, the steering of the vehicle (delta), and the acceleration / deacceleration (or throttling)


### 1.2 Timestep Length and Elapsed Duration (N & dt)###

I have selected N=50 and dt=0.05sec, which means that I am looking 2.5 seconds ahead. These are the values in the course material, 
and are a reasonable period of time assuming that conditions do not change much in that period. I would like to test N and dt
values that lead to up to 5 sec. In addition, the simulator´s latency is 100ms, so there is a lot of feedback during that 
5 sec period in case we need to udpate stuff

### 1.3 Polynomial Fitting and MPC Preprocessing ###

I have fitted a polynomial to the waypoints, and then evalute the polynomial on the current position of the car, to obtain
the CTE. I have used a third order polynomial, given that should be complex enough for most trajectories.

I have had trouble coverting the waypoints from the map/simulator coordinate system to the car coordinate system. I am not sure
if the formulas I have used for the waypoints are correct, and if the car position always translates to (0,0) in the car's
coordinate system. Any feedback here would be really useful.

I am also not sure if I am passing the correct values to the mpc_*_vals and next_*_vals variables, as I have not been able
to display the trajectories either. Additional feedback/help here would be useful

### 1.4 Model Predictive Control with Latency ###

I have implemented the MPC controller. I have implemented a cost function within the FG_eval class, and I have computed the 
state of the N-1 transitions within the trajectory. I have also implemented the Solve method, first initializing the constraints
and then calling the ipopt.solve method itself, to have the hessians/jacobians computed for me. I have basically used the code
from the MPC quizz.
