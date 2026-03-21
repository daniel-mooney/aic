import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import numpy as np

from superviser.planner import RRT, RRTStar
from superviser.planner.collision_checker import StraightCollisionChecker, RectangularObstacle2d
from superviser.planner.sampler import State, UniformEuclieanSampler
from superviser.planner.indexer import KDTree


class RRTTester(Node):
    def __init__(self) -> None:
        super().__init__("debug_rrt_node")

        self._collision_checker = StraightCollisionChecker()
        self._sampler = UniformEuclieanSampler(
            low=np.array([0.0, 0.0]),
            high=np.array([10.0, 10.0]),
        )

        self._indexer = KDTree(
            dim=2,
            distance_metric=lambda a, b: np.linalg.norm(a - b),
            axis_dist_metric=lambda a, b, _: abs(a - b),
        )

        # self._rrt = RRT(
        #     self._collision_checker,
        #     self._sampler,
        #     self._indexer,
        #     max_radius=0.1,
        # )

        self._rrt = RRTStar(
            self._collision_checker,
            self._sampler,
            self._indexer,
            join_radius=0.3,
            update_radius=0.6,
        )

        # ROS setup
        self.marker_id = 0

        qos = rclpy.QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._marker_pub = self.create_publisher(Marker, "visualization_marker", qos)

        self.setup_timer = self.create_timer(0.2, self.configure_env)
        self.run_timer = self.create_timer(1.0, self.find_path)

        self.get_logger().info("Setup RRTTester")

    def configure_env(self) -> None:
        self.configure_tf()

        # Create boundard and obstacles
        boundary_marker = Marker()

        boundary_marker.header.frame_id = "base_link"
        boundary_marker.header.stamp = self.get_clock().now().to_msg()

        boundary_marker.id = self.marker_id
        self.marker_id += 1
        boundary_marker.type = Marker.LINE_STRIP
        boundary_marker.action = Marker.ADD

        boundary_marker.points = [
                    Point(x=0.0, y=0.0),
                    Point(x=0.0, y=10.0),
                    Point(x=10.0, y=10.0),
                    Point(x=10.0, y=0.0),
                    Point(x=0.0, y=0.0),
                ]
        
        boundary_marker.pose.orientation.w = 1.0

        boundary_marker.color.r = 0.75
        boundary_marker.color.g = 0.4
        boundary_marker.color.b = 0.2
        boundary_marker.color.a = 1.0

        boundary_marker.scale.x = 0.05
        self._marker_pub.publish(boundary_marker)
        # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        
        # Add obstacles
        obstacle = RectangularObstacle2d(
            x_span=(4.0, 6.0),
            y_span=(2.0, 8.0),
        )

        self._collision_checker.add_obstacle(obstacle)

        # Create cube marker
        obs1_marker = Marker()

        obs1_marker.header.frame_id = "base_link"
        obs1_marker.header.stamp = self.get_clock().now().to_msg()

        obs1_marker.id = self.marker_id
        self.marker_id += 1
        obs1_marker.type = Marker.CUBE
        obs1_marker.action = Marker.ADD

        obs1_marker.pose.position.x = 5.0
        obs1_marker.pose.position.y = 5.0
        obs1_marker.pose.position.z = 0.05

        obs1_marker.pose.orientation.w = 1.0

        obs1_marker.scale.x = 2.0
        obs1_marker.scale.y = 6.0
        obs1_marker.scale.z = 0.1

        obs1_marker.color.r = 1.0
        obs1_marker.color.a = 1.0

        self._marker_pub.publish(obs1_marker)
        
        self.setup_timer.cancel()


    def configure_tf(self) -> None:
        # broadcaster cannot go out of scope immediately, keep it around
        self._broadcaster = StaticTransformBroadcaster(self)
        
        # Create frame at origin
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self._broadcaster.sendTransform(tf)

    def draw_point(self, point: np.ndarray) -> None:
        x, y = point

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.id = self.marker_id
        self.marker_id += 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0

        size = 0.05
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self._marker_pub.publish(marker)

    def draw_edge(self, a: np.ndarray, b: np.ndarray) -> None:
        x1, y1 = a
        x2, y2 = b

        line = Marker()
        line.header.frame_id = "base_link"
        line.header.stamp = self.get_clock().now().to_msg()

        line.id = self.marker_id
        self.marker_id += 1

        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.pose.orientation.w = 1.0

        line.points = [
            Point(x=x1,y=y1),
            Point(x=x2, y=y2),
        ]

        line.scale.x = 0.03

        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0

        self._marker_pub.publish(line)

    def draw_path(self, path: list[State]) -> None:
        points = [Point(x=xi, y=yi) for xi, yi in path]

        # Send points message
        point_marker = Marker()
        point_marker.header.frame_id = "base_link"
        point_marker.header.stamp = self.get_clock().now().to_msg()

        point_marker.id = self.marker_id
        self.marker_id += 1

        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD

        point_marker.pose.orientation.w = 1.0

        point_marker.points = points

        size = 0.05
        point_marker.scale.x = size
        point_marker.scale.y = size
        point_marker.scale.z = size

        point_marker.color.r = 1.0
        point_marker.color.g = 1.0
        point_marker.color.b = 1.0
        point_marker.color.a = 1.0

        self._marker_pub.publish(point_marker)

        # Send lines message
        path_marker = Marker()
        path_marker.header.frame_id = "base_link"
        path_marker.header.stamp = self.get_clock().now().to_msg()

        path_marker.id = self.marker_id
        self.marker_id += 1

        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.pose.orientation.w = 1.0
        path_marker.points = points

        path_marker.scale.x = 0.03

        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 1.0
        path_marker.color.a = 1.0

        self._marker_pub.publish(path_marker)

    def find_path(self) -> None:
        start = np.array([1.0, 5.0])
        goal = np.array([9.0, 4.0])

        self.get_logger().info("Searching for plan...")
        path = self._rrt.plan(start, goal)
        self.get_logger().info(f"Found plan (len={len(path)})")

        self.draw_path(path)

        self.run_timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    tester = RRTTester()
    rclpy.spin(tester)

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
