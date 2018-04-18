# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Reflection

Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. The models used in MPC are generally intended to represent the behavior of complex dynamical systems.
In this project we modify the task of following a trajectory as an optimization problem of selecting the set of actuator values that result in the lowest cost.

### Model
First I get the following information from the message from the simulator:
* ptsx (Array) - The global x positions of the waypoints.
* ptsy (Array) - The global y positions of the waypoints.
* psi (float) - The orientation of the vehicle in radians 
* x (float) - The global x position of the vehicle.
* y (float) - The global y position of the vehicle.
* steering_angle (float) - The current steering angle in radians.
* throttle (float) - The current throttle value [-1, 1].


#### Polynomial Fitting and MPC Preprocessing
The server returns waypoints using the map's coordinate system, which is different than the car's coordinate system. I have transformed these waypoints to make it easier to both display them and to calculate the CTE and Epsi values for the MPC. This was done in lines 114-119 of `main.cpp` file. This is done by substituting in  the rotation translation matrix(homgeneous transformation matrix) equations. 
First I translate from map coordinates to vehicle coordinate using:
* x = waypoint_x[i] - px
* y = waypoint_y[i] - py
where px,py is the position of car.
Then a rotation is applied with the heading angle of the car psi on these points in the reverse direction.
* transformed_x = x * cos(-psi) - y * sin(-psi)
* transformed_y = x * sin(-psi) + y * cos(-psi)

This makes it such that the car is at the origin. Hence px and py can be considered as 0,0 from now.
Then I used the polyfit function to fit a third degree polynomial to the transformed waypoints to get it's coefficients. 
Now I used the polyeval function to find the cross track error cte as just the `polyeval(coeffs,0.0)`. and Error in heading angle as `-atan(polyeval_derivative(coeffs,0.0))` where the function `polyeval_derivative()` calculates the first order derivative of the linear equation.


#### Model Predictive Control with Latency
After this I added the term for dealing latency by using the model equations with a delta_time of 0.1 (100 milliseconds) to predict the new state after 100ms has passed.
* new_x = x + v*cos(psi)*delta_time
* new_y = y + v*sin(psi)*delta_time
* new_psi = psi + (v/Lf) * -delta * delta_time
* new_v = v + a * delta_time
* new_cte = cte + v * sin(epsi) * timedelta;
* new_epsi = epsi + (v/Lf) * (-delta) * timedelta;
Here I take -delta since the delta from simulator is in reverse.

I then passed these new values as the state parameters for the Solve() function.

#### Model Solve function
To start off I set the N and dt values as 25 and 0.05 as suggested in the quizzes. I then set the number of model variables (includes both states and inputs) as `N * 6 + (N - 1) * 2`, as there are 6 state variables and 2 actuators.
Then I set the initial value of the independent variables as zero except the initial state which were set to values obtained from the State variable that was passed to the function.
Then I set all non-actuators upper and lowerlimits to the max negative and positive values. The upper and lower limits of delta are were set to -25 and 25 degrees (values in radians(-0.436332,0.436332)). Finally I set the Acceleration/decceleration upper and lower limits to -1, 1. 

Then I set the Lower and upper limits for the constraints 0 except for the initial state. For the initial state the constraints were set to the original state values x,y psi, v, cte and epsi.

After this I setup the FG_eval object that computes objective and constraints by passing the coeffs. This is used in the `ipopt::solve()` function.

#### Setting up Cost in FG_eval class
Here The cost is stored is the first element of `fg`. We initialize it to zero and add the costs multiplied by corresponding weights.

First I added the cost based on the reference state as follows:
square(cte), square(epsi) and square(v-reference_velocity)
Next I added the cost on delta and a to minimise the use of the actuators. square(a) and square(delta). This ensured that the actuators are used only when necessary and minimally.
In order to smoothen out the use of the actuators and avoid any jerks or unsafe moves and minimize the value gap between sequential actuations, I finally added the cost of change in actuator values square(delta[t+1]-delta[t]) and square(a[t+1]-a[t]).
Then using the model equations I predicted the values for each of the states for the next N timesteps and set them to corresponding variables.
##### Weight of cost and Timestep Length and Elapsed Duration (N & dt) tuning

Initially I kept the weights of the costs to 1 except change in delta which was set to 500 as mentioned in the quiz. The reference velocity was kept at 40 and the N and dt as 25 and 0.05 as mentioned above. This immediately gave a very good result where the car was following the track perfectly. Hence I tried increasing the reference velocity to 60. This immediately caused the car to flip over quite badly. I varied the values of the cost for cte and epsi to 2, 5, 10 and 100, 200 and 1000, but increasing that was only giving worser results. So I set it back to 1 and I tried the costs 10, 50 ,100, 150 and 200 for use of actuator delta. 100 150 and 200 improved the results but the car was still flipping over. I also tried different values of N and dt but they did not give any considerable improvements.

Hence I decided to decrease the reference velocity to 55.
I used the weights 1,1,1,1,1,500,1 again for cte,epsi,v-ref_v,delta,a,delta_change,a_change respectively. Here also the car performed poorly on turns after the bride. But playing with values of 700 for change in delta and 100 to 200 for delta significantly improved the results. The car crossed the turn after the bridge but failed on the next turn. I decided to use 100 and 700 for delta and change in delta respectively. I found that the car was overshooting due to large accelerations even at turns. So I added a weigh of 20 to the cost of acceleration. This gave a very good path for the car to follow. I was satisfied with these values ( cte 1, epsi 1, v-ref_v 1, delta 100, a 20, change in delta 700, change in a 1). Further increase in any of these weights gave no further improvements.

I then tried changing the N and dt to (20, 0.04), (20, 0.06), (30, 0.1) etc. While some of these showed no further improvements than what I already had, some of these completely caused car to crash. Hence I stuck with the original values for N and dt as (25, 0.05).

#### Setting the steering value and Predicted path
In the solve function from MPC.cpp I returned an array with values at index 0 and 1 set to delta and a. and the remaining values were set to the predicted x,y positions for N timesteps.

I set the steering angle as `-steer_value/deg2rad(25)` and a directly to acceleration value obtained. I also set the mpc_x_vals and mpc_y_vals to the predicted x and y values to show the predicted path as green line in the simulator.


