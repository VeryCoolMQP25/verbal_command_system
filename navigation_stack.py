import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
import os
import json
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Room coordinates 
        json_path = os.path.join(os.path.dirname(__file__), '../Robot-GUI/public/Unity_coords.json')
        with open(json_path, 'r') as file:
            self.room_coordinates = json.load(file)
        

    def navigate(self, room_number, floor_number):
        floor_key = f"floor_{floor_number}"
        self.get_logger().info(f"Attempting navigation to floor: {floor_key}, room: {room_number}")

        if floor_key in self.room_coordinates:
            if room_number in self.room_coordinates[floor_key]:
                coordinates = self.room_coordinates[floor_key][room_number]
                
                # Create goal message
                goal_msg = PoseStamped()
                
                # Set timestamp
                current_time = self.get_clock().now()
                goal_msg.header.stamp = current_time.to_msg()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position = Point(x=float(coordinates["x"]), y=float(coordinates["y"]), z=float(coordinates["z"]))
                goal_msg.pose.orientation = Quaternion(x=float(0.0), y=float(0.0), z=float(coordinates["orientationZ"]), w=float(coordinates["orientationW"]))
                self.goal_publisher.publish(goal_msg)
                # self.goal_publisher.publish(goal_msg)
                # self.goal_publisher.publish(goal_msg)

                self.get_logger().info("Goal published!")
                time.sleep(0.5)
                
                return True
            else:
                self.get_logger().error(f"Room {room_number} not found.")
        else:
            self.get_logger().error(f"Floor {floor_key} not found.")
        
        return False

def main():
    rclpy.init()
    nav_node = NavigationNode()
    
    try: #testing 
        room_num = "Elevator"
        floor = "1"
        success = nav_node.navigate(room_num, floor)
        if success:
            rclpy.spin_once(nav_node)
        else:
            nav_node.get_logger().error("Navigation failed!")
    
    except KeyboardInterrupt:
        pass
    finally:

        if rclpy.ok():
            nav_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()