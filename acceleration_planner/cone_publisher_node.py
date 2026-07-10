import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseArray, Pose

from .data_loader import CSVLoader


class ConePublisherNode(Node):
    def __init__(self):
        super().__init__("cone_publisher")

        self.declare_parameter("csv_path", "car_map_data.csv")
        self.declare_parameter("publish_rate", 1.0)  
        self.declare_parameter("frame_id", "map")

        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pub_yellow = self.create_publisher(PoseArray, "cones/yellow", qos)
        self.pub_blue = self.create_publisher(PoseArray, "cones/blue", qos)
        self.pub_red = self.create_publisher(PoseArray, "cones/red", qos)

        self.cones = CSVLoader(csv_path).get_cones()

        self.get_logger().info(
            f"Cones carregados de '{csv_path}' — "
            f"amarelos: {len(self.cones.yellow)}, "
            f"azuis: {len(self.cones.blue)}, "
            f"vermelhos: {len(self.cones.red)}"
        )

        period = 1.0 / rate if rate > 0 else 1.0
        self.timer = self.create_timer(period, self.publish_cones)

    def _to_pose_array(self, points) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_cones(self):
        self.pub_yellow.publish(self._to_pose_array(self.cones.yellow))
        self.pub_blue.publish(self._to_pose_array(self.cones.blue))
        self.pub_red.publish(self._to_pose_array(self.cones.red))


def main(args=None):
    rclpy.init(args=args)
    node = ConePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
           
            pass


if __name__ == "__main__":
    main()
