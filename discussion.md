# Model Predictive Control (MPC) Project

## The Model

The model consists of the vehicle's state, the actuators that can affect the state, and the equations describing how the actuators affect the state.

The vehicle state consists of the x- and y-coordinates, the heading, the velocity, the cross-track error (cte), and the heading error (epsi). I fit a third-order polynomial f (having coefficients coeffs) to the waypoints to get the trajectory I would like the vehicle to follow. This is used to compute the initial cte as `f(x) - y` (in code: `polyeval(coeffs, x) - y`) and the initial epsi as `heading - arctan(f'(x))` (in code: `heading - atan(coeffs[1] + coeffs[2] * x + coeffs[3] * pow(x, 2))`. Note that the initial cte and epsi are computed after converting from global coordinates to vehicle coordinates (see "Polynomial Fitting and MPC Preprocessing" below), so in the code, `x`, `y`, and `heading` are all 0.0, and these zero-terms were only included in the code to make it more obvious which equations were being applied.

The actuators are steering and acceleration. Steering has a physical limit of -25 degrees to 25 degrees (with counterclockwise steering being negative). Acceleration can take values from -1.0 to 1.0.

From an initial state of [x0, y0, psi0, v0, cte0, epsi0], given actuations delta (steering) and a (acceleration), after a timestep of dt, the new state [x1, y1, psi1, v1, cte1, epsi1] is given by the following equations:

```
  x1 = x0 + v0 * cos(psi0) * dt;
  y1 = y0 + v0 * sin(psi0) * dt;
  psi1 = psi0 + v0 / Lf * delta * dt;
  v1 = v0 + a * dt;
  cte1 = (f(x0) - y0) + (v0 * sin(epsi0) * dt);
  epsi1 = (psi0 - arctan(f'(x0)) + (v0 / Lf * delta * dt);
```

Here, Lf is the distance from the center of gravity of the vehicle to the front wheels. In code, `f(x0) - y0` and `psi0 - arctan(f'(x0))` are analogous to the initial cte and initial epsi code examples given above.

## Timestep Length and Elapsed Duration (N & dt)

The time horizon N * dt sets the amount of trajectory information the model is able to capture. More trajectory information is good because it provides more information for choosing the best set of actuations. To maximize the time horizon, N and dt should both be large. However, N cannot be too large, or else the IPOPT optimization problem will take too long to complete. And dt should not be too large either, or else too much time will have elapsed between timesteps, which leads to worse approximations of the desired trajectory.

Thus, dt was set to 0.1 (see "Model Predictive Control with Latency" below for more discussion on this choice), and N was set to 7. In practice, this was able to successfully drive the car around the track (without causing apparent lag in the simulator), whereas smaller values of N led to difficulties.

(I noticed there was a `max_cpu_time` option to IPOPT, which I believe was intended to set a hard cap on long-running optimization problems. This did not appear to be doing what it advertised on my 8-core machine. Please see the long comment in the MPC.cpp code for further discussion.)

Other values tried:

Before any weight tuning of cost function terms:
N = 25; dt = 0.05 -- The car swerves erratically.
N = 25; dt = 0.1 -- The car drives straight into the lake and the cost blows up.
N = 25; dt = 0.2 -- The car drives erratically and also ends up in the lake.
N = 5; dt = 0.1 -- The car hugs the right side of the track and is never able to make it toward the yellow trajectory.

After some weight tuning:
N = 10; dt = 0.2 -- The car follows the trajectory, but before too long the green trajectory gets erratic and the car drives off the track.
N = 10; dt = 0.15 -- This does an even better job following the trajectory, but can't make it past the first curve before going off track.
N = 10; dt = 0.1 -- The car is able to drive around the track, with a top speed of 35 mph.
N = 10; dt = 0.05 -- The car is able to drive quite a distance but oscillates heavily.

## Polynomial Fitting and MPC Preprocessing

Before running the MPC procedure, I convert the waypoints and vehicle state from global coordinates to vehicle coordinates using the following equations:

```
  car_ptsx[i] = (cos(-psi) * (ptsx[i] - px)) - (sin(-psi) * (ptsy[i] - py));
  car_ptsy[i] = (sin(-psi) * (ptsx[i] - px)) + (cos(-psi) * (ptsy[i] - py));
```

Here, `ptsx` and `ptsy` are the given waypoints in global coordinates, `px` and `py` are the vehicle's location in global coordinates, and `psi` is the vehicle's direction with respect to the global coordinate x- and y-axes. `car_ptsx` and `car_ptsy` are the waypoints in vehicle coordinates. Waypoints are updated sequentially in a loop.

Note that after this conversion, the vehicle x-coordinate, y-coordinate, and heading are all 0, and the x- and y-values making up the returned MPC vehicle trajectory are already in vehicle coordinates.

(After the MPC procedure, the steering angle delta is normalized to the range [-1, 1] by dividing by 25 degrees. I also flip left and right to match the simulator expectation by multiplying by -1. The acceleration a is consumed as-is.)

## Model Predictive Control with Latency

I deal with latency in 2 ways:

(1) I set dt to the latency (0.1 seconds). Setting dt to a value smaller than the latency is not very helpful for following waypoints because the actuations corresponding to the first timestep may already be stale by the time the latency has elapsed. More importantly, smaller dt values mean that the time horizon of N * dt is smaller, which provides a smaller window of trajectory information to the vehicle, which could have been useful for choosing actuations that are better in the long run. This is good because a short-sighted set of actuations may set off an oscillation that may be impossible to recover from. On the other hand, setting dt to a value larger than the latency allows even more time to elapse between actuations, which only worsens the apparent latency.

(2) I add weights to the cost function to heavily penalize "jerky" driving that can, over the duration of the latency, send the car to a dramatically different pose that is hard to reover from. Specifically, I apply large weights of 500 and 150, respectively, to the `CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)` steering smoothing term and `CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2)` acceleration smoothing term of the cost function. These weights dampen wobbling. (Aside: When I was playing with N = 10, I was also applying a small weight of 20 to the `CppAD::pow(vars[a_start + t + 1], 2)` acceration magnitude cost term to make it harder to achieve a speed of 45 mph, since exceeding 45 mph will cause the car to apply the brakes, and this was hard to deal with smoothly.)

