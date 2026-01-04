
import rclpy
from rclpy.node import Node
import heapq
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import yaml


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        
        # Parametri
        self.declare_parameter('map_file', 'moja_karta.pgm')
        self.declare_parameter('resolution', 0.5)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('viz_delay', 0.0001)  # Reduced for faster visualization
        
        map_file = self.get_parameter('map_file').value
        self.resolution = self.get_parameter('resolution').value
        self.frame_id = self.get_parameter('frame_id').value
        self.visualization_delay = self.get_parameter('viz_delay').value
        
        # Publisheri
        self.explored_pub = self.create_publisher(MarkerArray, '/explored_nodes', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        
        # Subscriber za kliknute tocke
        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        self.start_point = None
        self.goal_point = None
        self.maze = None
        self.origin = None
        
        # Load map
        self.load_map(map_file)
        
        self.get_logger().info("Dijkstra Planner initialized")
        self.get_logger().info(f"Map size: {self.maze.shape}")
        self.get_logger().info("Click 'Publish Point' in RViz2 to set START (first click) and GOAL (second click)")
        
    def load_map(self, map_file):
        yaml_file = map_file.replace('.pgm', '.yaml')

        with open(yaml_file, 'r') as f:
            map_yaml = yaml.safe_load(f)

        self.resolution = map_yaml['resolution']
        self.origin = map_yaml['origin']  # [x, y, yaw]

        image_path = map_yaml['image']
        self.maze = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        self.maze = np.flipud(self.maze)
        if self.maze is None:
            self.get_logger().error(f"Failed to load map image {image_path}")
            return

        self.maze = (self.maze < 10).astype(np.uint8)
    
    def world_to_grid(self, x, y):
        c = int((x - self.origin[0]) / self.resolution)
        r = int((y - self.origin[1]) / self.resolution)
        return r, c
    
    def grid_to_world(self, grid_pos):
        r, c = grid_pos
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (r + 0.5) * self.resolution
        return x, y
    
    def is_valid_point(self, r, c):
        """Check if grid point is valid and free"""
        rows, cols = self.maze.shape
        if 0 <= r < rows and 0 <= c < cols:
            return self.maze[r, c] == 0
        return False
    
    def clicked_point_callback(self, msg):
        """Handle clicked points from RViz2"""
        x, y = msg.point.x, msg.point.y
        r, c = self.world_to_grid(x, y)
        
        if not self.is_valid_point(r, c):
            self.get_logger().warn(f"Clicked point ({x:.2f}, {y:.2f}) -> grid ({r}, {c}) is invalid or occupied!")
            return
        
        if self.start_point is None:
            self.start_point = (r, c)
            self.get_logger().info(f"START set at grid ({r}, {c}), world ({x:.2f}, {y:.2f})")
            self.publish_goal_markers()
        elif self.goal_point is None:
            self.goal_point = (r, c)
            self.get_logger().info(f"GOAL set at grid ({r}, {c}), world ({x:.2f}, {y:.2f})")
            self.publish_goal_markers()
            
            self.get_logger().info("Starting path planning...")
            self.plan(self.start_point, self.goal_point)
        
            self.start_point = None
            self.goal_point = None
        else:
            self.start_point = (r, c)
            self.goal_point = None
            self.get_logger().info(f"START reset at grid ({r}, {c}), world ({x:.2f}, {y:.2f})")
            self.publish_goal_markers()
    
    def publish_goal_markers(self):
        """Publish markers for start and goal points"""
        marker_array = MarkerArray()
        
        if self.start_point:
            start_marker = Marker()
            start_marker.header.frame_id = self.frame_id
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "goals"
            start_marker.id = 0
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            
            x, y = self.grid_to_world(self.start_point)
            start_marker.pose.position.x = x
            start_marker.pose.position.y = y
            start_marker.pose.position.z = 0.2
            start_marker.pose.orientation.w = 1.0
            
            start_marker.scale.x = self.resolution * 3.0
            start_marker.scale.y = self.resolution * 3.0
            start_marker.scale.z = self.resolution * 3.0
            
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            
            marker_array.markers.append(start_marker)
        
        if self.goal_point:
            goal_marker = Marker()
            goal_marker.header.frame_id = self.frame_id
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "goals"
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            
            x, y = self.grid_to_world(self.goal_point)
            goal_marker.pose.position.x = x
            goal_marker.pose.position.y = y
            goal_marker.pose.position.z = 0.2
            goal_marker.pose.orientation.w = 1.0
            
            goal_marker.scale.x = self.resolution * 3.0
            goal_marker.scale.y = self.resolution * 3.0
            goal_marker.scale.z = self.resolution * 3.0
            
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            
            marker_array.markers.append(goal_marker)
        
        self.goal_marker_pub.publish(marker_array)
    
    def visualize_explored_node(self, node, node_id, color=(0.0, 1.0, 1.0, 0.5)):
        #Vizualizacija istrazenih cvorova kao sfere
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "explored_nodes"
        marker.id = node_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        x, y = self.grid_to_world(node)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = self.resolution * 0.9
        marker.scale.y = self.resolution * 0.9
        marker.scale.z = self.resolution * 0.9
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.lifetime.sec = 0
        
        return marker
    
    def visualize_path(self, path):
        #Vizualizacija planirane putanje kao linijskog markera
        if not path:
            return
        
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = self.resolution * 0.8
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for node in path:
            point = Point()
            x, y = self.grid_to_world(node)
            point.x = x
            point.y = y
            point.z = 0.05
            marker.points.append(point)
        
        marker.lifetime.sec = 0
        self.path_marker_pub.publish(marker)
        
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for node in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            x, y = self.grid_to_world(node)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def dijkstra(self, start, goal):
        #Dijkstra algoritam
        rows, cols = self.maze.shape
        
        distances = np.full((rows, cols), np.inf, dtype=np.float32)
        visited = np.zeros((rows, cols), dtype=bool)
        prev = np.full((rows, cols, 2), -1, dtype=np.int32)
        
        pq = [(0, start)]
        distances[start] = 0
        
        explored_markers = MarkerArray()
        node_id = 0
        
        neighbor_offsets = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]], dtype=np.int32)
        
        self.get_logger().info(f"Starting Dijkstra from {start} to {goal}")
        
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            r, c = current_node
            
            if visited[r, c]:
                continue
            
            visited[r, c] = True
            
            marker = self.visualize_explored_node(current_node, node_id)
            explored_markers.markers.append(marker)
            
            # Publishanje cvorova
            if len(explored_markers.markers) >= 200:  
                self.explored_pub.publish(explored_markers)
                time.sleep(0.1) 
                explored_markers = MarkerArray()
            
            node_id += 1
            
            if current_node == goal:
                self.get_logger().info("Goal reached!")
                if explored_markers.markers:
                    self.explored_pub.publish(explored_markers)
                break
            
            for dr, dc in neighbor_offsets:
                nr, nc = r + dr, c + dc
                
                if (0 <= nr < rows and 0 <= nc < cols and 
                    self.maze[nr, nc] == 0 and not visited[nr, nc]):
                    
                    new_distance = current_distance + 1
                    
                    if new_distance < distances[nr, nc]:
                        distances[nr, nc] = new_distance
                        prev[nr, nc] = [r, c]
                        heapq.heappush(pq, (new_distance, (nr, nc)))
        
        if explored_markers.markers:
            self.explored_pub.publish(explored_markers)
        
        # Rekonstrukcija putanje
        path = []
        node = goal
        while node is not None and node != (-1, -1):
            path.append(node)
            r, c = node
            pr, pc = prev[r, c]
            if pr == -1:
                break
            node = (pr, pc)
        
        path = path[::-1]
        
        if path and path[0] == start:
            self.get_logger().info(f"Path found with {len(path)} nodes")
            self.visualize_path(path)
            return path
        else:
            self.get_logger().warn("No path found!")
            return []
    
    def plan(self, start, goal):
        self.get_logger().info(f"Planning from {start} to {goal}")
        
        # Izbriši prethodne markere
        clear_marker = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        clear_marker.markers.append(marker)
        self.explored_pub.publish(clear_marker)
        self.path_marker_pub.publish(marker)
        
        time.sleep(0.1)
        
        # Pokreni Dijkstra algoritam
        start_time = time.time()
        path = self.dijkstra(start, goal)
        end_time = time.time()
        
        self.get_logger().info(f"Planning took {end_time - start_time:.3f} seconds")
        return path


def main(args=None):
    rclpy.init(args=args)
    
    planner = DijkstraPlanner()
    
    planner.get_logger().info("Ready! Use 'Publish Point' tool in RViz2:")
    planner.get_logger().info("  - First click: Set START (green sphere)")
    planner.get_logger().info("  - Second click: Set GOAL (red sphere) and start planning")
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()