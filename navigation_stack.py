import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
import os
import json 

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Room coordinates
        json_path = os.path.join(os.path.dirname(__file__), '../public/Unity_coords.json')
        with open(json_path, 'r') as file:
            self.room_coordinates = json.load(file)

    def navigate(self, room_number, floor_number):
        """Navigate to a specific room on a specific floor"""
        floor_key = f"floor_{floor_number}"
        self.get_logger().info(f"Navigating to floor: {floor_key}, room: {room_number}")
        
        if floor_key in self.room_coordinates:
            if room_number in self.room_coordinates[floor_key]:
                coordinates = self.room_coordinates[floor_key][room_number]
                
                # Create and publish goal message
                goal_msg = PoseStamped()
                current_time = self.get_clock().now()
                goal_msg.header.stamp = current_time.to_msg()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position = Point(x=float(coordinates["x"]), y=float(coordinates["y"]), z=float(coordinates["z"]))
                goal_msg.pose.orientation = Quaternion(x=float(0.0), y=float(0.0), z=float(coordinates["orientationZ"]), w=float(coordinates["orientationW"]))
                self.goal_publisher.publish(goal_msg)
                
                self.get_logger().info(f"Navigating to {room_number} on floor {floor_number}. Goal published.")
                
            else:
                self.get_logger().error(f"Room {room_number} not found.")
        else:
            self.get_logger().error(f"Floor {floor_key} not found.")

def main():
    rclpy.init()
    nav_node = NavigationNode()
    try:
        # For testing
        nav_node.navigate("Stairs", "1")        
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()