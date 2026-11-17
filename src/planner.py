import math
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt

#to do
#get end pose from cone coordinates
#make a separate planner class
#make a node class
#make init
#implement boundary check
@dataclass
class Pose2D:
    x: float
    y: float
    theta: float   


def wrap(a: float) -> float:
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a



def find_waypoints(start: Pose2D,
         end: Pose2D,
         steps_per_arc: int = 40) -> List[Pose2D]:
   
   

#1. transform end pose into frame with start pose at 0,0
#2. calculate turning radius and arc angle
#4. create waypoints along this path
#5. repeat for second arc
#6. transform back to world coor


    #transform into start frame
    dx = end.x - start.x
    dy = end.y - start.y
    c = math.cos(start.theta)
    s = math.sin(start.theta)

    xL =  c*dx + s*dy
    yL = -s*dx + c*dy

    #find turning radius and arc angle
    t = abs(yL) / xL
    theta_arc = 2 * math.atan(t)
    R = xL / (2 * math.sin(theta_arc))

    d1 = 1 if yL > 0 else -1
    d2 = -d1

    
    cx1 = 0
    cy1 = d1 * R
    phi0 = -d1 * math.pi/2

    #create waypoints
    local_pts = []

    for i in range(steps_per_arc):
        u = i / (steps_per_arc - 1)
        phi = phi0 + d1 * theta_arc * u
        x = cx1 + R * math.cos(phi)
        y = cy1 + R * math.sin(phi)
        heading = d1 * theta_arc * u
        local_pts.append(Pose2D(x, y, heading))

    mid = local_pts[-1]# store midpoint for second arc

    #repeat for second arc

    hx = math.cos(mid.theta)
    hy = math.sin(mid.theta)
    nx = -hy
    ny = hx

    cx2 = mid.x + nx * d2 * R
    cy2 = mid.y + ny * d2 * R

    phi1 = math.atan2(mid.y - cy2, mid.x - cx2)

    for i in range(1, steps_per_arc):
        u = i / (steps_per_arc - 1)
        phi = phi1 + d2 * theta_arc * u
        x = cx2 + R * math.cos(phi)
        y = cy2 + R * math.sin(phi)
        heading = mid.theta + d2 * theta_arc * u
        local_pts.append(Pose2D(x, y, heading))

    #convert to world
    path_world: List[Pose2D] = []
    for p in local_pts:
        wx = start.x + c*p.x - s*p.y
        wy = start.y + s*p.x + c*p.y
        wtheta = wrap(start.theta + p.theta)
        path_world.append(Pose2D(wx, wy, wtheta))

    return path_world

start_pose = Pose2D(x=0.0, y=0.0, theta=0.0)  # Starting at origin, facing along x-axis
end_pose = Pose2D(x=5.0, y=3.0, theta=math.pi / 4)  # End point, angled at 45 degrees

# Find waypoints
waypoints = find_waypoints(start=start_pose, end=end_pose, steps_per_arc=40)

# Print the waypoints
print("Generated Waypoints:")
for wp in waypoints:
    print(f"X: {wp.x:.2f}, Y: {wp.y:.2f}, Theta: {wp.theta:.2f}")

# Visualization using matplotlib
x_vals = [wp.x for wp in waypoints]
y_vals = [wp.y for wp in waypoints]

# Plotting the path
plt.figure(figsize=(8, 8))
plt.plot(x_vals, y_vals, marker='o', label="Waypoints Path")
plt.scatter([start_pose.x, end_pose.x], [start_pose.y, end_pose.y], color='red', label="Start/End")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Waypoints for Path Generation")
plt.legend()
plt.grid(True)
plt.show()