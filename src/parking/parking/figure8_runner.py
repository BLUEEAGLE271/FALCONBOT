#!/usr/bin/env python3
"""
Figure-8 Loop Runner with Live MPPI Tuning
==========================================
Publishes a green figure-8 path in RViz and drives the robot
around it continuously. Tune MPPI parameters live without restarting.

Usage:
  ros2 run parking figure8_runner
  
Tune live:
  ros2 param set /figure8_runner radius 1.5
  ros2 param set /figure8_runner speed 0.3
  ros2 param set /figure8_runner num_points 32
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class Figure8Runner(Node):
    def __init__(self):
        super().__init__('figure8_runner')

        # --- TUNABLE PARAMETERS ---
        # Change these live with: ros2 param set /figure8_runner <name> <value>
        self.declare_parameter('radius',     0.6)   # metres — lobe radius of figure-8
        self.declare_parameter('num_points', 24)    # waypoints — more = smoother path
        self.declare_parameter('speed',      0.3)   # not used by Nav2 directly, for reference
        self.declare_parameter('cx',         0.0)   # centre X of figure-8 in map frame
        self.declare_parameter('cy',         0.0)   # centre Y of figure-8 in map frame
        self.declare_parameter('frame',      'map') # reference frame

        self.add_on_set_parameters_callback(self._on_param_change)

        # --- ROS SETUP ---
        self.nav_client  = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.path_pub    = self.create_publisher(Path,   '/figure8_path',    10)
        self.marker_pub  = self.create_publisher(Marker, '/figure8_markers', 10)

        # State
        self._goal_handle   = None
        self._loop_count    = 0
        self._running       = False
        self._path_dirty    = True  # regenerate path when params change

        # Publish the visual path at 2Hz so RViz always shows it
        self.create_timer(0.5,  self._publish_path_visual)

        # Wait for Nav2 then start
        self.create_timer(2.0, self._try_start)

        self.get_logger().info(
            "Figure-8 Runner ready. Waiting for Nav2...\n"
            "Tune live: ros2 param set /figure8_runner radius 1.5"
        )

    # ------------------------------------------------------------------
    # Parameter change callback — triggers path regeneration
    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        for p in params:
            if p.name in ('radius', 'num_points', 'cx', 'cy', 'frame'):
                self._path_dirty = True
                self.get_logger().info(
                    f"Parameter '{p.name}' changed to {p.value} — "
                    f"path will regenerate on next loop."
                )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Waypoint generation — Lemniscate of Bernoulli (true figure-8)
    # ------------------------------------------------------------------
    def _generate_waypoints(self):
        radius     = self.get_parameter('radius').value
        num_points = int(self.get_parameter('num_points').value)
        cx         = self.get_parameter('cx').value
        cy         = self.get_parameter('cy').value
        frame      = self.get_parameter('frame').value

        poses = []
        for i in range(num_points):
            t = 2.0 * math.pi * i / num_points

            # Lemniscate parametric equations
            denom = 1.0 + math.sin(t) ** 2
            x = cx + radius * math.cos(t) / denom
            y = cy + radius * math.sin(t) * math.cos(t) / denom

            # Tangent direction for heading
            dt = 2.0 * math.pi / num_points
            t2 = t + dt
            denom2 = 1.0 + math.sin(t2) ** 2
            x2 = cx + radius * math.cos(t2) / denom2
            y2 = cy + radius * math.sin(t2) * math.cos(t2) / denom2

            dx  = x2 - x
            dy  = y2 - y
            yaw = math.atan2(dy, dx)

            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)

        return poses

    # ------------------------------------------------------------------
    # RViz visualisation — green path line + numbered waypoint markers
    # ------------------------------------------------------------------
    def _publish_path_visual(self):
        frame  = self.get_parameter('frame').value
        poses  = self._generate_waypoints()
        now    = self.get_clock().now().to_msg()

        # Green Path
        path             = Path()
        path.header.stamp     = now
        path.header.frame_id  = frame
        path.poses            = poses
        self.path_pub.publish(path)

        # Waypoint spheres — white dots along the path
        marker              = Marker()
        marker.header.stamp    = now
        marker.header.frame_id = frame
        marker.ns              = 'figure8_waypoints'
        marker.id              = 0
        marker.type            = Marker.SPHERE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = 0.06
        marker.scale.y         = 0.06
        marker.scale.z         = 0.06
        marker.color.r         = 1.0
        marker.color.g         = 1.0
        marker.color.b         = 1.0
        marker.color.a         = 0.8
        for pose in poses:
            marker.points.append(pose.pose.position)
        self.marker_pub.publish(marker)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def _try_start(self):
        """Called once 2s after startup. Waits for Nav2 then begins."""
        if self._running:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().info("Waiting for Nav2 navigate_through_poses...")
            return
        self.get_logger().info("✅ Nav2 ready — starting figure-8 loop!")
        self._running = True
        self._send_loop()

    def _send_loop(self):
        if not self._running:
            return

        poses = self._generate_waypoints()
        self._path_dirty = False
        self._loop_count += 1

        self.get_logger().info(
            f"🔄 Starting loop #{self._loop_count} | "
            f"radius={self.get_parameter('radius').value}m | "
            f"waypoints={len(poses)}"
        )

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_accepted_callback)

    def _goal_accepted_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            # Retry after 2s
            self.create_timer(2.0, lambda: self._send_loop())
            return

        self._goal_handle = handle
        self.get_logger().info("✅ Goal accepted — robot is running figure-8.")

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._loop_complete_callback)

    def _loop_complete_callback(self, future):
        """Called when the robot finishes one loop — immediately starts the next."""
        try:
            result = future.result()
            self.get_logger().info(
                f"✅ Loop #{self._loop_count} complete. "
                f"{'Path regenerated.' if self._path_dirty else 'Starting next loop.'}"
            )
        except Exception as e:
            self.get_logger().warn(f"Loop ended with: {e} — restarting.")

        # Small pause then restart — allows parameter changes to take effect
        self.create_timer(0.5, self._start_next_loop)

    def _start_next_loop(self):
        self._send_loop()


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Runner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()