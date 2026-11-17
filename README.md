# GEM-ParallelPark
Code for autonomous parallel parking of the GEM e2 Vehicle (ECE 484 Final Project).
Note: This final project was a joint work between myself, Caleb Peach (NetID: calebrp2), 
James Lee (jl212), Aartha Tavshikar (arthat2), and Dhatri Parakal (dhatrip2).

The code supports autonomous self-driving of the GEN e2 Vehicle for Parallel Parking. 
- Lidar and Camera perception assist with parking space detection, dimension computation, and obstacle detection for safety
- A custom planner generates a waypoint trajectory for the vehicle dependent on the dimensions of the parking space and the current pose of the vehicle relative to the center of the parking space.
- A PID controller executes the generated waypoint trajectory. Obstacles detected by LiDar in the path of the vehicle act as a gate on the PID controller - if an obstacle is detected, then the controller sends only the brake signal to the vehicle.
