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
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
import scipy.ndimage as ndimage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum

class State(Enum):
    WAITING_FOR_SERVER = 1
    SEARCHING = 2
    NAVIGATING = 3

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
        self.declare_parameter('blacklist_radius', 0.5)
        
        map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            1)
            
        self.viz_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 1)
        self.target_pub = self.create_publisher(PoseStamped, '/exploration/selected_target', 1)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.last_map = None
        self.processing = False
        
        self.state = State.WAITING_FOR_SERVER
        self.blacklist = []
        self.current_goal_handle = None
        self.current_goal_pose = None
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.is_shutting_down = False
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(self.get_parameter('update_interval').value, self.timer_callback)
        self.get_logger().info('Frontier Explorer Initialized')

    def map_callback(self, msg):
        self.last_map = msg

    def timer_callback(self):
        if self.state == State.WAITING_FOR_SERVER:
            if self.nav_client.server_is_ready():
                self.get_logger().info('Nav2 Server is ready. Starting exploration.')
                self.state = State.SEARCHING
            else:
                self.get_logger().info('Waiting for Nav2 Server...', throttle_duration_sec=5.0)
            return

        if self.last_map is None:
            return
        
        if self.processing:
            return
            
        self.processing = True
        try:
            # We process the map even when navigating to update visualization
            # But we only send a goal if in SEARCHING state
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
                
            # Centroid (may be in obstacles or unknown space)
            cy_mean, cx_mean = indices.mean(axis=0)
            
            # Validate candidate goal against free/navigable space:
            # Snap centroid to the nearest actual frontier cell (which is guaranteed free)
            dists = np.hypot(indices[:, 0] - cy_mean, indices[:, 1] - cx_mean)
            best_idx = np.argmin(dists)
            cy, cx = indices[best_idx]
            
            map_x = cx * resolution + origin_x
            map_y = cy * resolution + origin_y
            
            distance = math.hypot(map_x - robot_x, map_y - robot_y)
            
            # Filter unreachable/too close
            if distance < 0.2:
                continue
                
            # Filter blacklisted locations: blacklist candidate goals
            is_blacklisted = False
            for bx, by in self.blacklist:
                if math.hypot(map_x - bx, map_y - by) < self.blacklist_radius:
                    is_blacklisted = True
                    break
            if is_blacklisted:
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
            
            # Define goal orientation pointing towards the target
            yaw = math.atan2(best_target[1] - robot_y, best_target[0] - robot_x)
            
            # Publish target pose
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = best_target[0]
            pose.pose.position.y = best_target[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.target_pub.publish(pose)

            if self.state == State.SEARCHING:
                self.send_navigation_goal(pose)

    def send_navigation_goal(self, pose):
        self.state = State.NAVIGATING
        # Save goal position so callbacks can reference it without accessing goal_handle.request
        # (rclpy ClientGoalHandle does not expose the original request object)
        self.current_goal_pose = pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f'Sending goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        # Optionally log distance remaining
        # feedback = feedback_msg.feedback
        # self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected. Blacklisting location.')
            pose = self.current_goal_pose.pose
            self.blacklist.append((pose.position.x, pose.position.y))
            self.state = State.SEARCHING
            # Explicitly rescore
            self.timer_callback()
            return
            
        self.get_logger().info('Navigation goal accepted.')
        self.current_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().status
        pose = self.current_goal_pose.pose
        self.current_goal_handle = None
        self.current_goal_pose = None
        
        if self.is_shutting_down:
            return

        # SUCCEEDED = 4, CANCELED = 5, ABORTED = 6
        if status == 4:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().warn(f'Navigation failed with status {status}. Blacklisting location.')
            self.blacklist.append((pose.position.x, pose.position.y))
            
        self.state = State.SEARCHING
        # Explicitly rescore
        self.timer_callback()

    def destroy_node(self):
        self.is_shutting_down = True
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling active navigation goal before shutdown...')
            self.current_goal_handle.cancel_goal_async()
        super().destroy_node()

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
