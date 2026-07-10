import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker, MarkerArray

from . import config as cfg
from .planner import (
    build_trajectory,
    make_initial_car_state,
    step_simulation,
    decide_phase,
    CarState,
)


class PlannerNode(Node):
    def __init__(self):
        super().__init__("acceleration_planner")

        self.declare_parameter("frame_id", "map")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.yellow = None
        self.blue = None
        self.red = None
        self.traj = None
        self.car_state = None
        self.sim_timer = None

        self.create_subscription(PoseArray, "cones/yellow", self._cb_yellow, qos)
        self.create_subscription(PoseArray, "cones/blue", self._cb_blue, qos)
        self.create_subscription(PoseArray, "cones/red", self._cb_red, qos)

        self.pub_pose = self.create_publisher(PoseStamped, "car/pose", 10)
        self.pub_phase = self.create_publisher(String, "car/phase", 10)
        self.pub_speed = self.create_publisher(Float32, "car/speed", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "planner/markers", qos)

        self.get_logger().info("À espera dos 3 topics de cones (yellow/blue/red)...")

    @staticmethod
    def _from_pose_array(msg: PoseArray):
        return [(p.position.x, p.position.y) for p in msg.poses]

    def _cb_yellow(self, msg):
        if self.yellow is None:
            self.yellow = self._from_pose_array(msg)
            self._try_start()

    def _cb_blue(self, msg):
        if self.blue is None:
            self.blue = self._from_pose_array(msg)
            self._try_start()

    def _cb_red(self, msg):
        if self.red is None:
            self.red = self._from_pose_array(msg)
            self._try_start()

    def _try_start(self):
        if self.traj is not None or None in (self.yellow, self.blue, self.red):
            return


        self.traj = build_trajectory(self.yellow, self.blue, self.red)
        state = make_initial_car_state(self.traj)
        self.car_state = CarState(
            phase=decide_phase(state, self.traj),
            position=state.position,
            speed=state.speed,
            distance_to_finish=state.distance_to_finish,
        )

        self.get_logger().info(
            f"Trajetória calculada — {len(self.traj.midpoints)} midpoints, "
            f"eixo de avanço='{self.traj.advance_axis}'"
        )

        self._publish_trajectory_markers()
        self.sim_timer = self.create_timer(cfg.SIMULATION_DT, self._step)

    def _step(self):
        self.car_state = step_simulation(self.car_state, self.traj, cfg.SIMULATION_DT)
        self._publish_car_state()

        if self.car_state.phase == CarState.Phase.FINISHED:
            self.get_logger().info("Simulação terminada — carro parado na meta.")
            self.sim_timer.cancel()

    def _publish_car_state(self):
        x, y = self.car_state.position

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.orientation.w = 1.0
        self.pub_pose.publish(pose_msg)

        phase_msg = String()
        phase_msg.data = self.car_state.phase.name
        self.pub_phase.publish(phase_msg)

        speed_msg = Float32()
        speed_msg.data = float(self.car_state.speed)
        self.pub_speed.publish(speed_msg)

    def _publish_trajectory_markers(self):
        """Publica a linha de trajetória (midpoints) uma vez, para veres no rviz2."""
        array = MarkerArray()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "trajectory"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.15
        line.color.r, line.color.g, line.color.b, line.color.a = 0.18, 0.49, 0.20, 1.0

        for x, y in sorted(self.traj.midpoints, key=lambda p: p[1]):
            pt = Point()
            pt.x, pt.y, pt.z = float(x), float(y), 0.0
            line.points.append(pt)

        array.markers.append(line)
        self.pub_markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
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
