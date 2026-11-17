#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point

import numpy as np
import json


class LidarParkingGapDetector(Node):
    def __init__(self):
        super().__init__('lidar_parking_gap_detector')
        
        # Vehicle dimensions (GEM e2 default, in meters)
        self.declare_parameter('vehicle_length', 2.54)  # GEM e2 length
        self.declare_parameter('vehicle_width', 1.22)   # GEM e2 width
        self.declare_parameter('parking_clearance', 0.5)  # Extra space needed
        self.declare_parameter('max_parking_distance', 5.0)  # Max distance to consider
        self.declare_parameter('min_obstacle_points', 3)  # Min points to consider an obstacle
        self.declare_parameter('obstacle_gap_threshold', 0.3)  # Gap between points in same obstacle
        
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.parking_clearance = self.get_parameter('parking_clearance').value
        self.max_distance = self.get_parameter('max_parking_distance').value
        self.min_obstacle_points = self.get_parameter('min_obstacle_points').value
        self.obstacle_gap_threshold = self.get_parameter('obstacle_gap_threshold').value
        
        # Minimum required gap for parallel parking
        self.required_gap = self.vehicle_length + self.parking_clearance
        
        # Publisher for parking gap information
        self.gap_pub = self.create_publisher(String, 'parking_gaps', 10)
        
        # Subscriber to LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # TODO: fix with the lidar topic, i forgot
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f"LiDAR Parking Gap Detector Started.")
        self.get_logger().info(f"Vehicle length: {self.vehicle_length}m, Required gap: {self.required_gap}m")
    
    def scan_callback(self, msg):
        """Process LiDAR scan data to detect parking gaps"""
        
        # Convert polar coordinates to Cartesian
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings and points too far away
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max) & (ranges < self.max_distance)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[:len(ranges)][valid_mask]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid LiDAR points detected")
            return
        
        # Convert to Cartesian coordinates (x: forward, y: left)
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Focus on points to the side of the vehicle (perpendicular parking scenario)
        # Look for obstacles within a reasonable lateral range
        side_mask = (np.abs(y) > 0.5) & (np.abs(y) < self.max_distance) & (x > -2.0) & (x < 3.0)
        
        side_x = x[side_mask]
        side_y = y[side_mask]
        
        if len(side_x) == 0:
            self.get_logger().info("No obstacles detected in parking zone")
            return
        
        # Cluster points into obstacles (cars)
        obstacles = self.cluster_obstacles(side_x, side_y)
        
        if len(obstacles) < 2:
            self.get_logger().info(f"Only {len(obstacles)} obstacle(s) detected, need at least 2 to find gaps")
            return
        
        # Find gaps between consecutive obstacles
        gaps = self.find_gaps(obstacles)
        
        # Publish gap information
        self.publish_gaps(gaps)
    
    def cluster_obstacles(self, x, y):
        """Cluster LiDAR points into distinct obstacles (parked cars)"""
        if len(x) == 0:
            return []
        
        # Sort points by y-coordinate (lateral position)
        sorted_indices = np.argsort(y)
        x_sorted = x[sorted_indices]
        y_sorted = y[sorted_indices]
        
        obstacles = []
        current_cluster_x = [x_sorted[0]]
        current_cluster_y = [y_sorted[0]]
        
        for i in range(1, len(y_sorted)):
            # If this point is close to the previous one, add to current cluster
            dist = np.sqrt((x_sorted[i] - x_sorted[i-1])**2 + (y_sorted[i] - y_sorted[i-1])**2)
            
            if dist < self.obstacle_gap_threshold:
                current_cluster_x.append(x_sorted[i])
                current_cluster_y.append(y_sorted[i])
            else:
                # Save current cluster if it has enough points
                if len(current_cluster_x) >= self.min_obstacle_points:
                    obstacles.append({
                        'x': np.array(current_cluster_x),
                        'y': np.array(current_cluster_y),
                        'center_x': np.mean(current_cluster_x),
                        'center_y': np.mean(current_cluster_y),
                        'y_min': np.min(current_cluster_y),
                        'y_max': np.max(current_cluster_y)
                    })
                
                # Start new cluster
                current_cluster_x = [x_sorted[i]]
                current_cluster_y = [y_sorted[i]]
        
        # Don't forget the last cluster
        if len(current_cluster_x) >= self.min_obstacle_points:
            obstacles.append({
                'x': np.array(current_cluster_x),
                'y': np.array(current_cluster_y),
                'center_x': np.mean(current_cluster_x),
                'center_y': np.mean(current_cluster_y),
                'y_min': np.min(current_cluster_y),
                'y_max': np.max(current_cluster_y)
            })
        
        self.get_logger().info(f"Detected {len(obstacles)} obstacles")
        return obstacles
    
    def find_gaps(self, obstacles):
        """Find gaps between obstacles and determine if they're suitable for parking"""
        gaps = []
        
        # Sort obstacles by y-coordinate
        obstacles_sorted = sorted(obstacles, key=lambda obs: obs['center_y'])
        
        for i in range(len(obstacles_sorted) - 1):
            obs1 = obstacles_sorted[i]
            obs2 = obstacles_sorted[i + 1]
            
            # Calculate gap distance (lateral space between obstacles)
            gap_distance = obs2['y_min'] - obs1['y_max']
            
            # Calculate average forward position
            avg_x = (obs1['center_x'] + obs2['center_x']) / 2
            avg_y = (obs1['center_y'] + obs2['center_y']) / 2
            
            is_suitable = gap_distance >= self.required_gap
            
            gap_info = {
                'gap_id': i,
                'distance': float(gap_distance),
                'position_x': float(avg_x),
                'position_y': float(avg_y),
                'is_suitable': is_suitable,
                'clearance': float(gap_distance - self.vehicle_length) if is_suitable else None
            }
            
            gaps.append(gap_info)
            
            status = "SUITABLE" if is_suitable else "TOO SMALL"
            self.get_logger().info(
                f"Gap {i}: {gap_distance:.2f}m ({status}) at position ({avg_x:.2f}, {avg_y:.2f})"
            )
        
        return gaps
    
    def publish_gaps(self, gaps):
        """Publish gap information as JSON string"""
        suitable_gaps = [g for g in gaps if g['is_suitable']]
        
        output = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'vehicle_length': self.vehicle_length,
            'required_gap': self.required_gap,
            'total_gaps': len(gaps),
            'suitable_gaps': len(suitable_gaps),
            'gaps': gaps
        }
        
        msg = String()
        msg.data = json.dumps(output, indent=2)
        self.gap_pub.publish(msg)
        
        if suitable_gaps:
            self.get_logger().info(f"Found {len(suitable_gaps)} suitable parking spot(s)!")
        else:
            self.get_logger().info("No suitable parking spots found")


def main(args=None):
    rclpy.init(args=args)
    node = LidarParkingGapDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()