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

## Timestap Length & Elapsed Duration

The total forward looking time horizon was picked to be about 1 second. This was done by drawing the waypoint on the screen and looking for a line length that looked reasonably ahead to cover the upcoming road curvature changes. 

The values of `N=10` and `dt=0.1` were chosen because they yielded fast computations with stable responses. Choosing smaller dt values and larger N values resulted in more instability in the calculated cost function. 

## Polynomial Fitting & MPC Preprocessing

All the input data was transformed with respect to the car orientation and location. This made the car starting coordinate to be `(x,y,psi)=(0,0,0)`

The waypoint points were fitted to a 3rd degree polynomial. The starting cross track error was simply the fitted polynomial evalueated at `0`. The psi error was the angle of the tangent at `0` for the waypoint polynomial

## Model Predictive Control with Latency

The 100ms latency was handled by modeling the car 100ms into the future without any control input. It assumed that 


