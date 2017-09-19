- **The Model**: *Student describes their model in detail. This includes the state, actuators and update equations.*

The state model used in the solution includes the ego vehicle's x co-ordinate, y co-ordinate, heading angle and velocity. The cross track error and the heading angle error were also included. The vehicle control parameters are the throttle and the steering angle. 


- **Timestep Length and Elapsed Duration (N & dt)**: *Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

The value of N and dt were chosen so that their product equals 1 second. This is a reasonable duration for a vehicle in a real world scenario to look ahead especially if it is travelling at roughly the speeds at which the vehicle in the simulator is travelling (~60mph). If the product is too high then the vehicle may not follow the track closely enough whereas if the product is too low then the computational requirements will be too heavy.


- **Polynomial Fitting and MPC Preprocessing**: *A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

The waypoints were converted from global map co-ordinates to the vehicle's local co-ordinates so that the vehicle was at the origin. This was done to simplify calculations. 



- **Model Predictive Control with Latency**: *The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

Latency was incorporated into the software by deliberately putting the thread to sleep for 100ms in main.cpp:193. Latency was also simulated in MPC.cpp:90 by referring to actuation values one step behind the current time step.