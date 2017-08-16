## 1. Implementation ##

### 1.1 Model ###
I have used a kinematic model of the car. The state of the model is captured by 6 variables:
- x, y represent the position
- psi the orientation
- v the velocity
- cte the cross track error, or the error distance relative to the reference trajectory
- psie, the orientation error with respect to the reference trajectory

The equations used to update the next state based on the previous state are:

  x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt                                                                                                       
  y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt                                                                                                       
  psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt                                                                                                 
  v_[t+1] = v[t] + a[t] * dt                                                                                                                     
  cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt                                                                                           
  epsi[t+1] = epsi[t] + v[t] * delta[t] / Lf * dt 
  
  The trajectory error (cte) is computed by fitting a polynomial to the waypoints provided by the simulator (after a coordinate sytem change from
  map to car), evaluating the polynomial assuming a trajectory over the x axis, and comparing it with the y value:
  
  double cte = polyeval(coeffs, x) - y;
  
  The orientation error is computed by taking the derivative of the third order polynomial, which should be complex enough for most trajectory situations:
                                                                                              
  double epsi = psi - atan(coeffs[1] + 2 * x * coeffs[2] + 3 * coeffs[3] * pow(x,2));  // order 3    


The model also inludes two actuators, the steering of the vehicle (delta), and the acceleration / deacceleration (or throttling)


### 1.2 Timestep Length and Elapsed Duration (N & dt) ###

I have selected N=50 and dt=0.05sec, which means that I am looking 2.5 seconds ahead. These are the values in the course material, 
and are a reasonable period of time assuming that conditions do not change much in that period. I would like to test N and dt
values that lead to up to 5 sec. In addition, the simulator´s latency is 100ms, so there is a lot of feedback during that 
5 sec period in case we need to udpate stuff

### 1.3 Model Predictive Control with Latency ###

I have implemented the MPC controller. I have implemented a cost function within the FG_eval class, and I have computed the 
state of the N-1 transitions within the trajectory. I have also implemented the Solve method, first initializing the constraints and then calling the ipopt.solve method itself, to have the hessians/jacobians computed for me. I have basically used the code from the MPC quizz.

### 1.4 Latency compensation ###

In order to compensate for the 100ms delay that the simulator takes to implement the actuator values, I have predicted the state 100ms into the future, and passed this future state into the solver. I have computed the future state using the equations described in 1.1. I have obtained the steering angle and acceleration values from the simulator.

Given that the equations assume that a positive value of the steering angle means turning left, and a negative value means turning right, and in the simulator is actually the opposite, I have updated the equation for the orientation to:

psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt 

I have made this change both during latency compensation, and within the fg_eval function. I have then been able to pass the computed steering angle into the simulation, w/o any final sign change. In fact, using the latter did not work for me, and I only got it working when changing the sign in the psi equation.

Finally, the dt value using in latency compensation is 125ms, and not 100ms, to also account for the time it takes for the solver to compute a solution.

### 1.5 Parameter tuning ###

I have started with the common values N=10 and dt=0.1 (100ms timestep). I have also started with a reference velocity of 50 mph (and change it to meters per second per the latest reviewer suggestion, which made a big difference). I have also started with a weight of 400 for the steering angle in the cost function, with the rest of the weights at 1.0. This setting has siginificantly reduced the oscillations I was seeing.

I have been able to increase the velocity to 60 mph. Beyond that, oscillations start to be significant, taking the car out of the road in some curves.


