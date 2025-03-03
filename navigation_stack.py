import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
import asyncio
import websockets
import json
import threading

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Store connected clients - make this a class variable
        self.connected_clients = set()
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Room coordinates
        self.room_coordinates = { # I will load from JSON file, this is just temporarily copy-pasted in 
            "floor_1": {
                "Restrooms": { "x": -4, "y": 0.125, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH100": { "x": 2, "y": 19.2, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH150": { "x": 5.3, "y": 28.3, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH151": { "x": 3.49, "y": 11.4, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "Elevators": { "x": 4.64, "y": 3.15, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "Stairs": { "x": 1.7, "y": 31.3, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 }
            },
            "floor_2": {
                "Study_Area": { "x": 1.78, "y": 0.0, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "Stairs": { "x": 7.8, "y": 0.63, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Elevator": { "x": 5.0, "y": 26.7, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Restrooms": { "x": 13.7, "y": 28.9, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH200": { "x": 5.0, "y": -2.6, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH241": { "x": 5.5, "y": 8.6, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "UH220": { "x": 4.9, "y": 39.7, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "UH243": { "x": -1.7, "y": 20.8, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "UH235": { "x": 1.45, "y": 36.3, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 }
            },
            "floor_3": {
                "Elevator": { "x": 19.3, "y": 10.5, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Stairs": { "x": 47.9, "y": 10.8, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Restrooms": { "x": 17.4, "y": 18.5, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH300": { "x": 48.3, "y": 10.8, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH341": { "x": 39.6, "y": 9.5, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "UH320": { "x": 8.28, "y": 10.4, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "Study_Booths": { "x": 25.9, "y": 6.97, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "Study_Lounge": { "x": 50.2, "y": 4.98, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 }
            },
            "floor_4": {
                "UH400": { "x": 36.8, "y": 2.27, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH405": { "x": 26.1, "y": 1.73, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "UH420": { "x": 11.8, "y": 1.49, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "Elevator": { "x": 18.9, "y": 0.243, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Restrooms": { "x": 17.1, "y": 9.18, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "Study_Area": { "x": 48.7, "y": -0.122, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "Tech_Suites": { "x": 26.9, "y": -2.47, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "Stairs": { "x": 49.7, "y": 3.58, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 }
            },
            "floor_5": {
                "Study_Area": { "x": 71.7, "y": -4.84, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "Stairs": { "x": 66.2, "y": 0.8, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Elevator": { "x": 37.8, "y": -0.5, "z": 0.0, "orientationZ": 0.2, "orientationW": 0.98 },
                "Restrooms": { "x": 36.3, "y": 8.37, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH500": { "x": 55.6, "y": 1.02, "z": 0.0, "orientationZ": 0.00487, "orientationW": 0.99999 },
                "UH505": { "x": 43.4, "y": 1.1, "z": 0.0, "orientationZ": 0.1, "orientationW": 0.99 },
                "UH520": { "x": 18.4, "y": 0.5, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 },
                "Career_Development_Center": { "x": 16.7, "y": -6.11, "z": 0.0, "orientationZ": -0.1, "orientationW": 0.99 }
            }
        }
        
        # Start WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        self.get_logger().info("Navigation node initialized with WebSocket server")
    
    def start_websocket_server(self):
        """Start the WebSocket server in an asyncio event loop"""
        asyncio.run(self.run_websocket_server())
    
    async def run_websocket_server(self):
        """Run the WebSocket server"""
        server = await websockets.serve(self.handle_connection, "localhost", 8765)
        self.get_logger().info("WebSocket server started on ws://localhost:8765")
        await server.wait_closed()
    
    async def handle_connection(self, websocket, path):
        """Handle WebSocket connections"""
        # Register client
        self.connected_clients.add(websocket)
        try:
            # Keep connection open and handle messages
            async for message in websocket:
                # Process any incoming messages if needed
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # Unregister client
            self.connected_clients.remove(websocket)
    
    async def send_navigation_event(self, room):
        """Send navigation event to all connected WebSocket clients"""
        if self.connected_clients:
            # Create navigation message with the room parameter
            message = json.dumps({
                "type": "navigation",
                "room": room
            })
            # Send to all connected clients
            await asyncio.gather(
                *(client.send(message) for client in self.connected_clients)
            )
            self.get_logger().info(f"Navigation event sent for room: {room}")
        else:
            self.get_logger().warning("No WebSocket clients connected")
    
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
                
                # Send WebSocket event asynchronously
                asyncio.run_coroutine_threadsafe(
                    self.send_navigation_event(room_number),
                    asyncio.get_event_loop()
                )
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
        
        # Spin the node
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
