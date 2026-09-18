#!/usr/bin/env python3
"""
Frontier Exploration Node

Subscribes to:
  /map (nav_msgs/msg/OccupancyGrid)
  /tf (to get robot pose in map frame)

Publishes:
  /exploration/selected_target (geometry_msgs/msg/PoseStamped)
  /exploration/frontiers (visualization_msgs/msg/MarkerArray)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import scipy.ndimage as ndimage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_frontier_size', 15)
        self.declare_parameter('info_gain_radius', 2.0) # meters
        self.declare_parameter('info_gain_weight', 1.0)
        self.declare_parameter('distance_weight', 2.0)
        self.declare_parameter('size_weight', 0.5)
        self.declare_parameter('update_interval', 2.0)
        
        map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            1)
            
        self.viz_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 1)
        self.target_pub = self.create_publisher(PoseStamped, '/exploration/selected_target', 1)
        
        self.last_map = None
        self.processing = False
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(self.get_parameter('update_interval').value, self.timer_callback)
        self.get_logger().info('Frontier Explorer Initialized')

    def map_callback(self, msg):
        self.last_map = msg

    def timer_callback(self):
        if self.last_map is None:
            return
        
        if self.processing:
            return
            
        self.processing = True
        try:
            self.process_map(self.last_map)
        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")
        finally:
            self.processing = False

    def process_map(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        map_frame = map_msg.header.frame_id
        
        if width == 0 or height == 0:
            return
            
        # Get robot pose
        try:
            t = self.tf_buffer.lookup_transform(map_frame, self.base_frame, rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {map_frame} to {self.base_frame}: {ex}')
            return
            
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        
        # Free cells = 0, Unknown = -1
        free = (data == 0)
        unknown = (data == -1)
        
        # 8-connected kernel
        kernel = np.ones((3, 3), dtype=bool)
        
        # A cell is a frontier if it is FREE and adjacent to UNKNOWN
        unknown_dilated = ndimage.binary_dilation(unknown, structure=kernel)
        frontier_mask = free & unknown_dilated
        
        if not np.any(frontier_mask):
            self.get_logger().info("No frontiers found.")
            return
            
        # Cluster frontiers
        labeled_frontiers, num_features = ndimage.label(frontier_mask, structure=kernel)
        
        min_size = self.get_parameter('min_frontier_size').value
        info_radius_m = self.get_parameter('info_gain_radius').value
        info_radius_cells = int(info_radius_m / resolution)
        
        w_info = self.get_parameter('info_gain_weight').value
        w_dist = self.get_parameter('distance_weight').value
        w_size = self.get_parameter('size_weight').value
        
        best_score = -float('inf')
        best_target = None
        
        markers = MarkerArray()
        
        cluster_id = 0
        for i in range(1, num_features + 1):
            indices = np.argwhere(labeled_frontiers == i)
            size = len(indices)
            
            if size < min_size:
                continue
                
            # Centroid
            cy, cx = indices.mean(axis=0)
            map_x = cx * resolution + origin_x
            map_y = cy * resolution + origin_y
            
            distance = math.hypot(map_x - robot_x, map_y - robot_y)
            
            # Filter unreachable/too close
            if distance < 0.2:
                continue
                
            # Information gain
            y_min = max(0, int(cy - info_radius_cells))
            y_max = min(height, int(cy + info_radius_cells))
            x_min = max(0, int(cx - info_radius_cells))
            x_max = min(width, int(cx + info_radius_cells))
            sub_unknown = unknown[y_min:y_max, x_min:x_max]
            info_gain = np.sum(sub_unknown)
            
            # Score
            score = (w_info * info_gain) - (w_dist * distance) + (w_size * size)
            
            # Create visualization marker for this cluster
            marker = Marker()
            marker.header.frame_id = map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = cluster_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = resolution
            marker.scale.y = resolution
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            for pt in indices:
                p = Point()
                p.x = pt[1] * resolution + origin_x
                p.y = pt[0] * resolution + origin_y
                p.z = 0.0
                marker.points.append(p)
                
            markers.markers.append(marker)
            cluster_id += 1
            
            if score > best_score:
                best_score = score
                best_target = (map_x, map_y, size)
                
        # Delete unused markers from previous runs
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'frontiers'
        
        # Publish visualization
        self.viz_pub.publish(markers)
        
        if best_target is not None:
            self.get_logger().info(f"Selected Frontier: ({best_target[0]:.2f}, {best_target[1]:.2f}), score={best_score:.2f}, size={best_target[2]}")
            
            # Publish target pose
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = best_target[0]
            pose.pose.position.y = best_target[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.target_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
