#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        self.car_width = 0.35
        self.t_safe = 0.15
        self.wheelbase = 0.48
        
        self.max_speed = 3.0
        self.max_speed_2 = 4.0
        self.mid_speed_1 = 1.0
        self.mid_speed_2 = 2.0 #2.5
        self.mid_speed_3 = 3.0 #3.0
        self.min_speed = 1.6
        self.max_steering_angle = 1.0
        
        self.disparity_threshold = 0.08
        self.lookahead = 1.0

        self.min_distance = 0.1
        self.max_distance = 3.0

        self.prev_angle_min = None
        self.prev_angle_max = None
        self.prev_ranges_size = None
        self.angles = None
        self.front_indices = None
        self.angle_increment = None

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.filtered_scan_publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.get_logger().info('Pure Pursuit Node Initialized')

        self.min_gap_width = 0.45  # Minimum width needed to pass
        self.passing_angle_threshold = np.pi/4  # 30 degrees
        self.passing_speed_multiplier = 1.2  # 50% speed boost
        self.passing_cooldown = 0.01
        self.passing_mode = False
        self.last_passing_action_time = time.time()

        self.acceleration_threshold = 1.5 #Safe distance needed for full acceleration
        self.max_acceleration_multiplier = 1.2 # 20% speed boost for clear path

    def scan_callback(self, scan_msg):
        start_time = time.time()
        try:
            ranges = np.array(scan_msg.ranges)
            ranges = np.clip(ranges, self.min_distance, self.max_distance)

            # Check if we need to recompute angles and indices
            if (self.prev_angle_min != scan_msg.angle_min or
                self.prev_angle_max != scan_msg.angle_max or
                self.prev_ranges_size != len(ranges)):
                
                self.angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
                self.prev_angle_min = scan_msg.angle_min
                self.prev_angle_max = scan_msg.angle_max
                self.prev_ranges_size = len(ranges)

                self.front_indices = np.where(np.abs(self.angles) <= np.pi / 2)[0]
                self.angle_increment = scan_msg.angle_increment

            angles = self.angles

            disparities = self.find_disparities(ranges)
            filtered_ranges = self.extend_disparities(ranges.copy(), disparities)
            
            self.publish_filtered_scan(scan_msg, filtered_ranges)
            best_angle = self.find_best_angle(filtered_ranges)

            # Calculate base controls
            steering_angle = self.determine_steering_angle(best_angle)
            base_speed = self.determine_speed(filtered_ranges)

            # Check for passing opportunity first; If not passing, check for acceleration opportunity
            passing = self.check_passing_opportunity(filtered_ranges, self.angles, best_angle)
            accelerating = False if passing else self.check_acceleration_opportunity(filtered_ranges, self.angles)

            # Adjust speed based on mode
            if passing:
                speed = base_speed * self.passing_speed_multiplier
            elif accelerating:
                speed = base_speed * self.max_acceleration_multiplier
            else:
                # Normal speed adjustments based on angle
                angle_diff = abs(best_angle)
                if angle_diff > 0.3:
                    speed = self.mid_speed_1
                elif angle_diff > 0.2:
                    speed = self.mid_speed_2
                elif angle_diff > 0.1:
                    speed = self.mid_speed_3
                elif angle_diff > 0.05:
                    speed = base_speed
                else:
                    speed = self.max_speed_2

            self.publish_drive_command(steering_angle, speed)
            
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {str(e)}")
            self.publish_drive_command(0.0, 0.0)
        
        end_time = time.time()  
        elapsed_time = end_time - start_time  
        self.get_logger().info(f"scan_callback execution time: {elapsed_time:.6f} seconds")  

    def find_disparities(self, ranges):
        """Find range disparities using vectorized operations"""
        diffs = np.abs(np.diff(ranges))
        disparities = np.where(diffs > self.disparity_threshold)[0]
        return disparities

    def extend_disparities(self, ranges, disparities):
        """Extend disparities for safety"""
        angle_increment = self.angle_increment
        for idx in disparities:
            closer_idx = idx if ranges[idx] < ranges[idx + 1] else idx + 1
            further_idx = idx + 1 if closer_idx == idx else idx

            dist = ranges[closer_idx]
            angle_width = np.arctan((self.car_width / 2 + self.t_safe) / dist)
            num_extend = int(np.ceil(angle_width / angle_increment))

            if closer_idx > further_idx:
                start_idx = max(0, closer_idx - num_extend)
                ranges[start_idx:closer_idx] = dist
            else:
                end_idx = min(len(ranges) - 1, closer_idx + num_extend)
                ranges[closer_idx:end_idx] = dist
        
        return ranges

    def publish_filtered_scan(self, scan_msg, filtered_ranges):
        """Publish the disparity-masked scan as a LaserScan message"""
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = self.min_distance
        filtered_scan.range_max = self.max_distance
        filtered_scan.ranges = filtered_ranges.tolist()
        filtered_scan.intensities = scan_msg.intensities  
        
        self.filtered_scan_publisher.publish(filtered_scan)

    def find_best_angle(self, ranges):
        """Find best angle using gap-finding approach"""
        front_ranges = ranges[self.front_indices]
        front_angles = self.angles[self.front_indices]
        valid_mask = front_ranges > self.max_distance
        
        if not np.any(valid_mask):
            # Fallback to deepest point
            max_idx = np.argmax(front_ranges)
            return front_angles[max_idx]

        # Find the widest gap
        valid_indices = np.where(valid_mask)[0]
        gap_starts = np.where(np.diff(np.concatenate(([False], valid_mask, [False]))))[0]
        gap_lengths = gap_starts[1::2] - gap_starts[::2]
        
        if len(gap_lengths) > 0:
            widest_gap_idx = np.argmax(gap_lengths)
            start_idx = gap_starts[::2][widest_gap_idx]
            end_idx = gap_starts[1::2][widest_gap_idx]

            # Target the center of the widest gap
            center_idx = (start_idx + end_idx) // 2
            return front_angles[center_idx]
        else:
            # Fallback if no gaps found
            max_idx = np.argmax(front_ranges)
            return front_angles[max_idx]

    def determine_steering_angle(self, best_angle):
        """Calculate steering angle using pure pursuit"""
        lookahead_distance = self.lookahead
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(best_angle), lookahead_distance)
        
        # Limit steering angle
        return np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

    def determine_speed(self, ranges):
        """Determine speed based on front distance"""
        # Calculate index for front angle (0 radians)
        front_idx = int(round(-self.prev_angle_min / self.angle_increment))
        front_idx = max(0, min(len(ranges) - 1, front_idx))
        front_distance = ranges[front_idx]

        # Adjust speed based on distance
        if front_distance < 0.1:
            speed = 0.0  # Emergency stop
        elif front_distance < 5.0:
            speed = self.min_speed + (self.max_speed - self.min_speed) * (front_distance - 1.0) / 4.0
        else:
            speed = self.max_speed

        return speed

    def publish_drive_command(self, steering_angle, speed):
        """Publish drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_publisher.publish(drive_msg)

    def check_passing_opportunity(self, ranges, angles, best_angle):
        """Check if we can pass based only on gap width"""
        current_time = time.time()
        if current_time - self.last_passing_action_time < self.passing_cooldown:
            return False

        # Check gap width on both sides
        left_indices = np.where((angles >= np.pi/6) & (angles <= np.pi/3))[0]
        right_indices = np.where((angles <= -np.pi/6) & (angles >= -np.pi/3))[0]
        
        left_gap = np.min(ranges[left_indices]) if len(left_indices) > 0 else 0
        right_gap = np.min(ranges[right_indices]) if len(right_indices) > 0 else 0
        
        best_gap = max(left_gap, right_gap)
        if best_gap > self.min_gap_width and abs(best_angle) < self.passing_angle_threshold:
            passing_side = 'left' if left_gap > right_gap else 'right'
            self.passing_mode = True
            self.last_passing_action_time = current_time
            self.get_logger().info(f"Passing on {passing_side}! Gap: {best_gap:.2f}m")
            return True

        return False

    def check_acceleration_opportunity(self, ranges, angles):
        """Check if we can accelerate based on forward clearance"""
        if self.passing_mode:  # Don't check acceleration during passing
            return False

        front_indices = np.where(np.abs(angles) <= np.pi/6)[0]
        if len(front_indices) == 0:
            return False

        forward_distance = np.min(ranges[front_indices])
        
        if forward_distance > self.acceleration_threshold:
            self.get_logger().info(f"Accelerating! Clear distance: {forward_distance:.2f}m")
            return True

        return False

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
