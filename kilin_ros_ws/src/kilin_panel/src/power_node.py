from rclpy.node import Node
from kilin_msgs.msg import PowerCmdStamped, PowerStateStamped
from PyQt5 import QtWidgets, QtCore


# ---------- Custom Event ----------
class PowerStateUpdateEvent(QtCore.QEvent):
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, power_state):
        super().__init__(PowerStateUpdateEvent.EVENT_TYPE)
        self.power_state = power_state


class PowerNode(Node):
    """ROS2 node for publishing /power/command and subscribing /power/state."""

    def __init__(self, ui_ref=None):
        super().__init__('power_gui_node')

        # Publisher
        self.publisher = self.create_publisher(PowerCmdStamped, '/power/command', 10)

        # Subscriber
        self.subscriber = self.create_subscription(
            PowerStateStamped, '/power/state', self.power_state_callback, 10
        )

        self.seq = 0
        self.ui_ref = ui_ref

        # Throttle setup (10 Hz)
        self._last_update_ns = 0
        THROTTLE_HZ = 10
        self._THROTTLE_NS = int(1e9 / THROTTLE_HZ)

    # -------------------------------------------------------------
    # Publish power command
    # -------------------------------------------------------------
    def publish_power_command(self, digital, signal, power):
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

    # -------------------------------------------------------------
    # Power state callback (full data + throttle)
    # -------------------------------------------------------------
    def power_state_callback(self, msg: PowerStateStamped):
        """Receive full /power/state and push to UI at 10 Hz."""
        if not self.ui_ref:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_update_ns < self._THROTTLE_NS:
            return
        self._last_update_ns = now_ns

        try:
            # Gather complete state
            power_state = {
                "digital": msg.digital,
                "signal": msg.signal,
                "power":  msg.power,
                "clean":  msg.clean,
                "voltages": [
                    msg.v_0, msg.v_1, msg.v_2, msg.v_3, msg.v_4, msg.v_5,
                    msg.v_6, msg.v_7, msg.v_8, msg.v_9, msg.v_10, msg.v_11
                ],
                "currents": [
                    msg.i_0, msg.i_1, msg.i_2, msg.i_3, msg.i_4, msg.i_5,
                    msg.i_6, msg.i_7, msg.i_8, msg.i_9, msg.i_10, msg.i_11
                ],
            }

            # Send event to Panel
            QtWidgets.QApplication.postEvent(
                self.ui_ref,
                PowerStateUpdateEvent(power_state)
            )

        except Exception as e:
            self.get_logger().error(f"Power state callback error: {e}")
