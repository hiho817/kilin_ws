from rclpy.node import Node
from kilin_msgs.msg import MotorCmdStamped, LegCmd, MotorCmd, MotorStateStamped
from PyQt5 import QtWidgets, QtCore


# ---------- Custom Event ----------
class MotorStateUpdateEvent(QtCore.QEvent):
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, modules_state):
        super().__init__(MotorStateUpdateEvent.EVENT_TYPE)
        self.modules_state = modules_state


class MotorNode(Node):
    """ROS2 node for publishing /motor/command and subscribing /motor/state."""

    def __init__(self, ui_ref=None):
        super().__init__('motor_gui_node')
        self.pub_motor = self.create_publisher(MotorCmdStamped, '/motor/command', 10)
        self.sub_motor_state = self.create_subscription(
            MotorStateStamped, '/motor/state', self.motor_state_callback, 10
        )

        self.seq = 0
        self.ui_ref = ui_ref
        self._last_update_ns = 0
        self._THROTTLE_NS = 0 

    def publish_motor_command(self, modules):
        msg = MotorCmdStamped()
        msg.header.seq = self.seq
        msg.header.time = self.get_clock().now().to_msg()
        msg.header.frame_id = "motor_ui"

        def build_leg(panel):
            leg_msg = LegCmd()
            data = panel.get_leg_cmd()

            for name in ["hip", "steering", "hub"]:
                m = MotorCmd()
                d = data[name]
                m.position = d["position"]
                m.velocity = d["velocity"]
                m.torque = d["torque"]
                m.kp = d["kp"]
                m.ki = d["ki"]
                m.kd = d["kd"]

                mode_map = {
                    "rest": 0, "config": 1, "set zero": 2,
                    "hal calibrate": 3, "position mode": 4,
                    "velocity mode": 5, "torque mode": 6
                }
                m.motor_mode = mode_map.get(d["mode"].lower(), 0)

                setattr(leg_msg, name, m)
            return leg_msg

        msg.module_a = build_leg(modules["A"])
        msg.module_b = build_leg(modules["B"])
        msg.module_c = build_leg(modules["C"])
        msg.module_d = build_leg(modules["D"])

        self.pub_motor.publish(msg)
        self.seq += 1

    def motor_state_callback(self, msg: MotorStateStamped):
        if not self.ui_ref:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_update_ns < self._THROTTLE_NS:
            return
        self._last_update_ns = now_ns

        try:
            modules_state = {}
            for mod_name, leg in zip(
                ["A", "B", "C", "D"],
                [msg.module_a, msg.module_b, msg.module_c, msg.module_d]
            ):
                modules_state[mod_name] = {
                    "hip": (leg.hip.position, leg.hip.velocity, leg.hip.torque, leg.hip.motor_mode),
                    "steering": (leg.steering.position, leg.steering.velocity, leg.steering.torque, leg.steering.motor_mode),
                    "hub": (leg.hub.position, leg.hub.velocity, leg.hub.torque, leg.hub.motor_mode),
                }

            QtWidgets.QApplication.postEvent(
                self.ui_ref, MotorStateUpdateEvent(modules_state)
            )

        except Exception as e:
            self.get_logger().error(f"motor_state_callback error: {e}")
