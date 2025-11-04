from rclpy.node import Node
from kilin_msgs.msg import PowerCmdStamped, PowerStateStamped
from PyQt5 import QtWidgets, QtCore

# ---------- Custom Event ----------
class VoltageUpdateEvent(QtCore.QEvent):
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, voltage_value):
        super().__init__(VoltageUpdateEvent.EVENT_TYPE)
        self.voltage = voltage_value


# ---------- PowerNode ----------
class PowerNode(Node):
    """ROS2 node for publishing /power/command and subscribing /power/state."""

    def __init__(self, ui_ref=None):
        super().__init__('power_gui_node')
        self.publisher = self.create_publisher(PowerCmdStamped, '/power/command', 10)
        self.subscriber = self.create_subscription(
            PowerStateStamped,
            '/power/state',
            self.power_state_callback,
            10
        )
        self.seq = 0
        self.ui_ref = ui_ref

    def publish_power_command(self, digital, signal, power):
        """Publish PowerCmdStamped message."""
        msg = PowerCmdStamped()
        msg.header.seq = self.seq
        msg.header.time = self.get_clock().now().to_msg()

        msg.digital = digital
        msg.signal = signal
        msg.power = power
        msg.clean = False
        msg.trigger = False
        msg.steering_cali = False

        self.publisher.publish(msg)
        self.seq += 1
        self.get_logger().info(
            f"Published PowerCmd: D={int(digital)} S={int(signal)} P={int(power)}"
        )

    def power_state_callback(self, msg: PowerStateStamped):
        """Subscribe to /power/state and update voltage display."""
        try:
            v0 = msg.v_0
            if self.ui_ref:
                QtWidgets.QApplication.postEvent(self.ui_ref, VoltageUpdateEvent(v0))
        except Exception as e:
            self.get_logger().error(f"Power state callback error: {e}")
