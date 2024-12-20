#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

import random

from turtlesim.srv import Spawn, Kill 
from my_robot_interfaces.msg import TurtleArray 
from my_robot_interfaces.srv import CatchTurtle 


class TurtleSpawnNode(Node):
    def __init__(self):
        super().__init__("turtle_spawn_node")
        
        self.declare_parameter("turtle_spawn_period", 2.5)
        self.declare_parameter("alive_turtle_publish_period", 0.1)
        self.turtle_spawn_period_ = self.get_parameter("turtle_spawn_period").value
        self.alive_turtle_publish_period_ = self.get_parameter("alive_turtle_publish_period").value
        
        self.alive_turtles_ = {'x': [], 'y': [], 'theta': [], 'name': []}
        self.timer_spawn_ = self.create_timer(self.turtle_spawn_period_, self.call_spawn_server)
        
        self.alive_turtle_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.timer_publish_ = self.create_timer(self.alive_turtle_publish_period_, self.publish_alive_turtles)
        
        self.server_catch_turtle_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        
        self.get_logger().info("Turtle spawn node has been started.")

    
    def call_spawn_server(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for server spawn...")

        request = Spawn.Request()
        request.x = 1 + 9 * random.random()
        request.y = 1 + 9 * random.random()
        request.theta = 3.14 * random.random()
        if len(self.alive_turtles_['name']) == 0:
            request.name = "turtle_2"
        else:
            request.name = f"turtle_{int(self.alive_turtles_['name'][-1][7:])+1}"
        
        self.alive_turtles_['x'].append(request.x)
        self.alive_turtles_['y'].append(request.y)
        self.alive_turtles_['theta'].append(request.theta)
        self.alive_turtles_['name'].append(request.name)
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn))
        
        
    def call_kill_server(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("waiting for server kill...")

        request = Kill.Request()
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill))
        
        
    def callback_call_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle spawned")
            
        except Exception as e:
            self.get_logger().info(f"Service call failed {str(e)}")
            
    
    def callback_call_kill(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle killed")
        except Exception as e:
            self.get_logger().info(f"Service call failed {str(e)}")
            
            
    def callback_catch_turtle(self, request, response):
        self.get_logger().info(
            f"Incoming request\na: Catch turtle {request.name}")
        
        if request.name not in self.alive_turtles_['name']:
            response.success = False
            self.get_logger().info(f"Sending response: turtle caught {response.success}")
            return response
        
        self.call_kill_server(request.name)
        
        # remove the caught turtle from the list
        self.alive_turtles_['x'].pop(0)
        self.alive_turtles_['y'].pop(0)
        self.alive_turtles_['theta'].pop(0)
        self.alive_turtles_['name'].pop(0)
        
        response.success = True
        
        self.get_logger().info(f"Sending response: turtle caught {response.success}")
        
        return response
    
            
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.x = self.alive_turtles_['x']
        msg.y = self.alive_turtles_['y']
        msg.theta = self.alive_turtles_['theta']
        msg.name = self.alive_turtles_['name']
        
        self.alive_turtle_publisher_.publish(msg)
        # self.get_logger().info("List of alive turtles published.")
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
