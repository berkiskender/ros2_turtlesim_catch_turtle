#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial

import random

from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray 
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")
        
        self.declare_parameter("master_cmdvel_period", 0.1)
        self.declare_parameter("catch_dist_threshold", 0.75)
        self.master_cmdvel_period_ = self.get_parameter("master_cmdvel_period").value
        self.catch_dist_threshold_ = self.get_parameter("catch_dist_threshold").value
        
        self.alive_turtles_ = {'x': [], 'y': [], 'theta': [], 'name': []}
        self.turtle1_pose_ = Pose()
        
        self.subscriber_alive_turtles_ = self.create_subscription(
            TurtleArray, 'alive_turtles', self.callback_alive_turtles, 10)
        
        self.subscriber_turtle1_pose_ = self.create_subscription(
            Pose, 'turtle1/pose', self.callback_turtle1_pose, 10)
        
        self.publisher_turtle1_cmdvel_ = self.create_publisher(
            Twist, 'turtle1/cmd_vel', 10)
        self.timer_publish_cmdvel_ = self.create_timer(
            self.master_cmdvel_period_, self.publish_turtle1_cmdvel_)
    
        self.get_logger().info("Turtle controller node has been started.")
    
    
    # obtain alive turtles pose and name
    def callback_alive_turtles(self, msg):
        # self.get_logger().info(f"Received alive turtles: {msg.name}")
        
        self.alive_turtles_['x'] = msg.x
        self.alive_turtles_['y'] = msg.y
        self.alive_turtles_['theta'] = msg.theta
        self.alive_turtles_['name'] = msg.name
        # self.get_logger().info(
        #     'Alive turtles: ' + str(self.alive_turtles_['name']))


    # obtain turtle1 pose
    def callback_turtle1_pose(self, msg): 
        self.turtle1_pose_ = msg
        # print(f"turtle1 pose: {self.turtle1_pose_.x} {self.turtle1_pose_.y} {self.turtle1_pose_.theta}")
        if len(self.alive_turtles_['name']) > 0:
            dist = ((self.turtle1_pose_.x - self.alive_turtles_['x'][0])**2 + (self.turtle1_pose_.y - self.alive_turtles_['y'][0])**2)**0.5
            if dist < self.catch_dist_threshold_:
                # self.get_logger().info(f"turtle1 distance to target: {dist}")
                self.get_logger().info(
                    f"turtle1 is close to target turtle: {dist}, calling catch service for {self.alive_turtles_['name'][0]}")
                self.call_catch_turtle_server(self.alive_turtles_['name'][0])
                
                self.get_logger().info(
                    'Alive turtles: ' + str(self.alive_turtles_['name']))
        
    # publish cmd_vel for turtle1
    def publish_turtle1_cmdvel_(self):
        
        if len(self.alive_turtles_['name']) > 0:
            msg = Twist()
            
            dist_x = self.alive_turtles_['x'][0] - self.turtle1_pose_.x
            dist_y = self.alive_turtles_['y'][0] - self.turtle1_pose_.y
            distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

            # position
            msg.linear.x = 2 * distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.turtle1_pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff
            
            self.publisher_turtle1_cmdvel_.publish(msg)
            # self.get_logger().info(f"Cmd_vel: {msg.linear.x} {msg.linear.y} {msg.angular.z} is published for turtle1.")


    def call_catch_turtle_server(self, name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("waiting for server catch_turtle...")

        request = CatchTurtle.Request()
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle))
    
    
    def callback_call_catch_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle caught")
        except Exception as e:
            self.get_logger().info(f"Service call failed {str(e)}")

  
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
