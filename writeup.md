# Project 5 - Model Predictive Control

## The Model
The vehicle dynamics model from the class notes was used. They are defined in lines `152-161` of `MPC.cpp`.

`delta0` & `a0` are the actuators at time `t`.
```
// expected y value at x0
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
// expected car orientation (tangent angle at x0)
AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * x0 * x0));

// Setup the rest of the model constraints
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 = (psi0 + v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);
cte1 = ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
### Cost Model
The cost model considered the following 
* difference between reference state and target (cte, epsi, ref_v). These were the highest weighted costs
* Penalize too much actuator control
* Minimize the change in actuator control over progressive cycles

The cost model is defined in `MPC.cpp` line `97-114`. The weights are in line `12-19` of `MPC.cpp`.

## Timestap Length & Elapsed Duration

The total forward looking time horizon was picked to be about 1 second. This was done by drawing the waypoint on the screen and looking for a line length that looked reasonably ahead to cover the upcoming road curvature changes. 

The values of `N=10` and `dt=0.1` were chosen because they yielded fast computations with stable responses. Choosing smaller dt values (<0.075) and larger N (>20) values resulted in more instability in the calculated cost function.

--- Edit 

Based on review feedback, I reduce cte weight significantly. This helped reduce osciallation. Reducing the dt timestemp and increasing N to have a 1 second horizon `(dt=0.05, N=20)`, resulted in more stable following. Now the car doesn't aggressively try to over-correct if it is off center.

## Polynomial Fitting & MPC Preprocessing

All the input data was transformed with respect to the car orientation and location. This made the car starting coordinate to be `(x,y,psi)=(0,0,0)`

The waypoint points were fitted to a 3rd degree polynomial. The starting cross track error was simply the fitted polynomial evalueated at `0`. The psi error was the angle of the tangent at `0` for the waypoint polynomial

## Model Predictive Control with Latency

The latency was handled by using the motion model to predict the car into the future without any control input. 

This was done by timing how long it took from receiving a message from the car to when we send the updated control values. This includes the 100ms sleep timeout and the delay to optimize the cost function. 

Since psi starts out 0 because we've already transformed the coordinates with respect to the car, only the `x` and `psi` state values were transformed given the measured velocity and steering angle

```
x0 = x0 + v0 * lastDelayEstimate;
y0 = y0 + 0;
psi0 = psi0 + (v0/Lf) * delta * lastDelayEstimate;
v0 = v0 + 0; // no change assuming 0 acceleration (approx.)
cte0 = cte0 + v0 * sin(epsi0) * lastDelayEstimate;
epsi0 = epsi0 + (v0/Lf) * delta * lastDelayEstimate;
```

After taking into account the delay, there was some initial oscillation and instability in the response, but it stabilised and the car was able to drive smoothly around the track as can be [seen here](https://drive.google.com/open?id=0B2KVtEVE1lFjOEVfS2JOby15Tzg)

The car was driven around the track at 50mph. Attempting a 100mph run resulted in the car driving around the edges of the track. I attempted to tune the parameters to increase the penalties for cte and epsi deviation. Also made it more expensive to have higher throttle. That didn't reduce the car going over the track edges at 100mph. If there was more time, I would do a more thorough parameter search
