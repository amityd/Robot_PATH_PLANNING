#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile
import tf_transformations


class ExplorationNode(Node):
    def __init__(self):
        super().__init__("exploration_node")
        
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.map_callback, QoSProfile(depth=1))
        
        self.explored_nodes = set()  # Set of explored nodes
        self.frontier = set()        # Set of frontier nodes
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0

        self.timer = self.create_timer(10.0, self.explore)
    
    def map_callback(self, msg):
        # Update map dimensions and resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        self.get_logger().info(f"Map width{self.map_width} and Height{self.map_height}")
        
        # Update explored and frontier nodes based on the map's occupancy grid
        # self.update_graph_from_map(msg)
        
    def update_graph_from_map(self, map_msg):
        # Convert occupancy grid to a 2D array
        occupancy_array = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height))
        
        # Iterate through the array and update explored and frontier nodes
        for x in range(map_msg.info.width):
            for y in range(map_msg.info.height):
                occupancy_value = occupancy_array[x, y]
                if occupancy_value == 0:
                    self.explored_nodes.add((x, y))
                elif occupancy_value == -1:
                    self.frontier.add((x, y))
        
    def generate_goal_pose(self):
        if not self.frontier:
            goal_pose = self.choose_random_unexplored_point()
        else:
            goal_pose = self.choose_goal_based_on_algorithm()
        
        return goal_pose
    
    def choose_random_unexplored_point(self):
        unexplored_points = [(x, y) for x in range(self.map_width) for y in range(self.map_height)
                             if (x, y) not in self.explored_nodes and (x, y) not in self.frontier]
        if not unexplored_points:
            return None
        
        x, y = random.choice(unexplored_points)
        return self.convert_to_pose(x, y)
    
    def choose_goal_based_on_algorithm(self):
        current_node = random.choice(list(self.frontier))
        self.frontier.remove(current_node)
        self.explored_nodes.add(current_node)

        neighbors = self.get_neighbors(current_node)
        unexplored_neighbors = [neighbor for neighbor in neighbors if neighbor not in self.explored_nodes]

        if unexplored_neighbors:
            next_node = random.choice(unexplored_neighbors)
            self.frontier.add(next_node)
            return self.convert_to_pose(next_node[0], next_node[1])
        
        return None

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.map_width and 0 <= ny < self.map_height]

    
    def convert_to_pose(self, x, y):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose = NavigateToPose.Goal()
        pose.pose.header.frame_id = "map"
        pose.pose.pose.position.x = x * self.map_resolution
        pose.pose.pose.position.y = y * self.map_resolution
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = q_x
        pose.pose.pose.orientation.y = q_y
        pose.pose.pose.orientation.z = q_z
        pose.pose.pose.orientation.w = q_w
        
        return pose
    
    def explore(self):
        goal_pose = self.generate_goal_pose()
        if goal_pose:
            self.get_logger().info(f"Exploring to goal pose {goal_pose}")

            send_goal_future = self.navigator.send_goal_async(goal_pose)
            self.goal_handle = send_goal_future.result()

            if self.goal_handle:
                self.goal_result_future = self.goal_handle.get_result_async()
                self.wait_for_goal_completion()

    def wait_for_goal_completion(self):
        while not self.goal_result_future.done():
            pass

        goal_result = self.goal_result_future.result()
        
        if goal_result and goal_result.accepted:
            self.get_logger().info("Goal accepted!")
        else:
            self.get_logger().info("Goal rejected!")

        self.goal_handle = None
        self.goal_result_future = None



def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    
    try:
        rclpy.spin(exploration_node)
    except KeyboardInterrupt:
        pass
    
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
