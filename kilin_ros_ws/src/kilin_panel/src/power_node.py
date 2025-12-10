from rclpy.node import Node
from kilin_msgs.msg import PowerCmdStamped, PowerStateStamped
from PyQt5 import QtWidgets, QtCore


# ---------- Custom Event: Full Power State ----------
class PowerStateUpdateEvent(QtCore.QEvent):
    """
    Custom Qt event carrying the entire power state message.
    """
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, msg: PowerStateStamped):
        super().__init__(PowerStateUpdateEvent.EVENT_TYPE)

        # Basic power flags
        self.digital = msg.digital
        self.signal = msg.signal
        self.power = msg.power
        self.clean = msg.clean

        # Voltages (v_0 ~ v_11)
        self.voltages = [
            msg.v_0, msg.v_1, msg.v_2, msg.v_3,
            msg.v_4, msg.v_5, msg.v_6, msg.v_7,
            msg.v_8, msg.v_9, msg.v_10, msg.v_11
        ]

        # Currents (i_0 ~ i_11)
        self.currents = [
            msg.i_0, msg.i_1, msg.i_2, msg.i_3,
            msg.i_4, msg.i_5, msg.i_6, msg.i_7,
            msg.i_8, msg.i_9, msg.i_10, msg.i_11
        ]


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
        """Subscribe to /power/state and send UI update event."""
        try:
            if self.ui_ref:
                # Use one integrated event
                QtWidgets.QApplication.postEvent(
                    self.ui_ref,
                    PowerStateUpdateEvent(msg)
                )

        except Exception as e:
            self.get_logger().error(f"Power state callback error: {e}")
