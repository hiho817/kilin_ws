from PyQt5 import QtCore
from rclpy.node import Node

from kilin_msgs.msg import TriggerStamped


class TriggerStateUpdateEvent(QtCore.QEvent):
    EVENT_TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, enabled: bool):
        super().__init__(self.EVENT_TYPE)
        self.enabled = bool(enabled)


class TriggerNode(Node):
    def __init__(self, ui_ref, topic="/kilin/trigger"):
        super().__init__("kilin_trigger_node")
        self.ui_ref = ui_ref
        self.sub = self.create_subscription(
            TriggerStamped, topic, self._cb, 10
        )

    def _cb(self, msg: TriggerStamped):
        QtCore.QCoreApplication.postEvent(
            self.ui_ref,
            TriggerStateUpdateEvent(msg.enable)
        )