import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import math
import os
import time

class NavWaypointNode(Node):
    def __init__(self):
        super().__init__('nav_waypoints_node')

        # Publisher to Nav2 goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.create_subscription(NavigateToPose_FeedbackMessage,
                                 '/navigate_to_pose/_action/feedback',
                                 self.feedback_callback,
                                 10)
        self.create_subscription(PointStamped,
                                 '/clicked_point',
                                 self.clicked_point_callback,
                                 10)

        # Load initial waypoints from file
        pkg_path = os.path.dirname(os.path.realpath(__file__))
        waypoint_file = os.path.join(pkg_path, '../config/waypoints.txt')
        self.waypoints = self.load_waypoints(waypoint_file)

        # State
        self.current_goal_idx = 0
        self.goal_active = False
        self.goal_tolerance = 0.10  # meters
        self.last_distance = None

        # Start sending the first goal after 3s delay
        self.create_timer(3.0, self.try_publish_next_goal)

        self.get_logger().info("nav_waypoints node initialized.")
        self.get_logger().info("Waiting for Nav2 to start...")

    # -----------------------------
    # Load waypoints from text file
    # -----------------------------
    def load_waypoints(self, path):
        waypoints = []
        with open(path, 'r') as f:
            for line in f:
                vals = [float(x.strip()) for x in line.split(',')]
                if len(vals) == 4:
                    waypoints.append(vals)
        self.get_logger().info(f"Loaded {len(waypoints)} static waypoints.")
        return waypoints

    # -----------------------------
    # RViz clicked point handler
    # -----------------------------
    def clicked_point_callback(self, msg: PointStamped):
        """Append clicked RViz point as new waypoint"""
        x, y = msg.point.x, msg.point.y
        new_wp = [x, y, 0.0, 1.0]
        self.waypoints.append(new_wp)
        self.get_logger().info(f"Added waypoint from RViz: ({x:.2f}, {y:.2f})")

        # If no current goal active, start moving immediately
        if not self.goal_active:
            self.try_publish_next_goal()

    # -----------------------------
    # Feedback from Nav2
    # -----------------------------
    def feedback_callback(self, msg: NavigateToPose_FeedbackMessage):
        """Check if the robot is near the current goal"""
        if not self.goal_active:
            return
        try:
            current_pose = msg.feedback.current_pose.pose
            goal = self.waypoints[self.current_goal_idx - 1]
            dx = goal[0] - current_pose.position.x
            dy = goal[1] - current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.last_distance = distance

            # Within tolerance → mark goal complete
            if distance < self.goal_tolerance:
                self.get_logger().info(f"Reached waypoint {self.current_goal_idx} "
                                       f"(dist={distance:.2f} m)")
                self.goal_active = False
                time.sleep(1.5)
                self.try_publish_next_goal()
        except Exception as e:
            self.get_logger().warn(f"Feedback error: {e}")

    # -----------------------------
    # Publish next goal if ready
    # -----------------------------
    def try_publish_next_goal(self):
        """Send next waypoint to /goal_pose"""
        if self.goal_active:
            return  # Still moving

        if self.current_goal_idx < len(self.waypoints):
            x, y, z, w = self.waypoints[self.current_goal_idx]
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w

            self.goal_pub.publish(msg)
            self.current_goal_idx += 1
            self.goal_active = True
            self.get_logger().info(
                f"Sent goal {self.current_goal_idx}: ({x:.2f}, {y:.2f})")
        else:
            self.get_logger().info("All waypoints completed.")
            self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = NavWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
