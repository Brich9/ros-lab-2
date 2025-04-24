# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan  , 'diff_drive/scan', self.pose_callback, 10)
        # Front Laser distance reading
        self.pose = None
        # Left Laser distance reading
        self.pose_left  = None
        # Used for PID calculation, distance from left wall        
        self.left_distance = 1.15
        # PID tunes
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 1.0
        self.dt = 0.1
        # Previous left distance
        self.previous_left_distance = 0.0
        self.left_int = 0.0 # Not used but needed if integral is added
        self.prev_distance = 0.0 # Not used in final implementation

    def pose_callback(self, position):
        # Get laser info
        self.pose = position.ranges[0]
        self.pose_left = position.ranges[1]
        msg = Twist()
        # Make PID calculations
        distance_error_left = self.pose_left- self.left_distance
        left_prop = distance_error_left
        self.left_int = self.left_int + distance_error_left * self.dt
        left_derivative = (distance_error_left - self.previous_left_distance) / self.dt
        distance = self.Kp * left_prop + self.Kd * left_derivative + self.Ki * self.left_int
        
        # Move at a set rate and turn based on calculations
        msg.linear.x = 1.0
        msg.angular.z = distance
        # used for logging
        # self.get_logger().info(f"{self.pose_left}")

        
        # Specific case for entering the room
        if self.pose_left > 5.0 and self.pose_left < 8.0 :
            msg.linear.x = 0.1
            msg.angular.z = 0.5
        # Turn right if too close to the wall
        elif self.pose < 1.8:
            msg.linear.x = 0.10
            msg.angular.z = -0.6
        # Dont react to a sudden change or a really far wall
        elif self.pose_left - self.previous_left_distance > 7.0 or self.pose_left > 8.7:
            msg.linear.x = 1.0
            msg.angular.z = 0.0
        
        
        self.publisher_.publish(msg)

        self.previous_left_distance = distance_error_left
        self.prev_distance = distance


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
