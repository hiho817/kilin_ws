from rclpy.node import Node
from kilin_msgs.msg import MotorCmdStamped, LegCmd, MotorCmd, MotorStateStamped
from PyQt5 import QtWidgets, QtCore

# ---------- Custom Event ----------
class MotorStateUpdateEvent(QtCore.QEvent):
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, module_name, joint_name, pos, vel, tor):
        super().__init__(MotorStateUpdateEvent.EVENT_TYPE)
        self.module = module_name
        self.joint = joint_name
        self.pos = pos
        self.vel = vel
        self.tor = tor


class MotorNode(Node):
    """ROS2 node for publishing /motor/command and subscribing /motor/state."""

    def __init__(self, ui_ref=None):
        super().__init__('motor_gui_node')
        self.pub_motor = self.create_publisher(MotorCmdStamped, '/motor/command', 10)
        self.sub_motor_state = self.create_subscription(
            MotorStateStamped,
            '/motor/state',
            self.motor_state_callback,
            10
        )
        self.seq = 0
        self.ui_ref = ui_ref

    # -------------------------------------------------------------
    # Publish motor command
    # -------------------------------------------------------------
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

                if name == "hip":
                    leg_msg.hip = m
                elif name == "steering":
                    leg_msg.steering = m
                else:
                    leg_msg.hub = m
            return leg_msg

        msg.module_a = build_leg(modules["A"])
        msg.module_b = build_leg(modules["B"])
        msg.module_c = build_leg(modules["C"])
        msg.module_d = build_leg(modules["D"])

        self.pub_motor.publish(msg)
        self.seq += 1
        self.get_logger().info("Published /motor/command successfully.")

    # -------------------------------------------------------------
    # Subscribe motor/state
    # -------------------------------------------------------------
    def motor_state_callback(self, msg: MotorStateStamped):
        """Receive /motor/state and send GUI update events."""
        if not self.ui_ref:
            return

        modules = {
            "A": msg.module_a,
            "B": msg.module_b,
            "C": msg.module_c,
            "D": msg.module_d
        }

        try:
            for module_name, leg in modules.items():
                for joint_name, motor_state in zip(
                    ["hip", "steering", "hub"],
                    [leg.hip, leg.steering, leg.hub]
                ):
                    # 只抓取 position / velocity / torque
                    pos = motor_state.position
                    vel = motor_state.velocity
                    tor = motor_state.torque
                    QtWidgets.QApplication.postEvent(
                        self.ui_ref,
                        MotorStateUpdateEvent(module_name, joint_name, pos, vel, tor)
                    )
        except Exception as e:
            self.get_logger().error(f"motor_state_callback error: {e}")
