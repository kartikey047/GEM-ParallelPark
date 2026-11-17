import numpy as np
import matplotlib.pyplot as plt

# Initial position and orientation of the vehicle in the global frame
vehicle_position = np.array([0.0, 0.0, 0.0])  # [x, y, theta] where theta is the heading (orientation)

# Camera measurements representing the 4 corners of the parking space in the camera's frame
parking_edges = np.array([
    [2, 1],  # Top-left corner
    [4, 1],  # Top-right corner
    [4, 3],  # Bottom-right corner
    [2, 3]   # Bottom-left corner
])

def convert_to_global_frame(camera_edges, vehicle_position):
    x, y, theta = vehicle_position
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    transformed_points = []
    for edge in camera_edges:
        translated_edge = edge + np.array([x, y])
        global_edge = rotation_matrix @ translated_edge
        transformed_points.append(global_edge)
    
    return np.array(transformed_points)

def update_vehicle_position(current_position, dx, dy, dtheta):
    x, y, theta = current_position
    new_x = x + dx * np.cos(theta) - dy * np.sin(theta)
    new_y = y + dx * np.sin(theta) + dy * np.cos(theta)
    new_theta = theta + dtheta
    return np.array([new_x, new_y, new_theta])

def slam_process(initial_position, camera_edges, steps=10, move_x=0.5, move_y=0.0, rotate_theta=0.1):
    position = initial_position
    trajectory = [position]
    map_edges = [] 
    
    for _ in range(steps):
        global_edges = convert_to_global_frame(camera_edges, position)
        
        map_edges.append(global_edges)
        
        position = update_vehicle_position(position, move_x, move_y, rotate_theta)
        trajectory.append(position)
    
    return np.array(trajectory), np.array(map_edges)

vehicle_trajectory, mapped_edges = slam_process(vehicle_position, parking_edges)

def display_slam_results(trajectory, mapped_edges):
    plt.figure(figsize=(10, 10))
    
    plt.plot(trajectory[:, 0], trajectory[:, 1], label='Vehicle Path', color='blue', marker='o')
    
    for i, edges in enumerate(mapped_edges):
        plt.plot(edges[:, 0], edges[:, 1], label=f'Edges at step {i+1}', marker='x')
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.title('Vehicle Localization and Map Building')
    plt.grid(True)
    plt.show()

display_slam_results(vehicle_trajectory, mapped_edges)