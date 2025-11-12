from PyQt5 import QtWidgets
from mainwindow import Ui_MainWindow
from module_widget import ModulePanel
from power_node import PowerNode, VoltageUpdateEvent
from motor_node import MotorNode, MotorStateUpdateEvent
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import signal
import sys
import time


class KilinPanel(QtWidgets.QMainWindow, Ui_MainWindow):
    """Main GUI for Kilin control system."""

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # -------------------------------------------------------------
        # Initialize ROS2 and nodes
        # -------------------------------------------------------------
        rclpy.init(args=None)
        self.power_node = PowerNode(ui_ref=self)
        self.motor_node = MotorNode(ui_ref=self)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.power_node)
        self.executor.add_node(self.motor_node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # -------------------------------------------------------------
        # Insert module widgets
        # -------------------------------------------------------------
        self.module_a = ModulePanel("A")
        self.module_b = ModulePanel("B")
        self.module_c = ModulePanel("C")
        self.module_d = ModulePanel("D")
        self._insert_module(self.groupBox_module_a, self.module_a)
        self._insert_module(self.groupBox_module_b, self.module_b)
        self._insert_module(self.groupBox_module_c, self.module_c)
        self._insert_module(self.groupBox_module_d, self.module_d)

        # -------------------------------------------------------------
        # Power control states
        # -------------------------------------------------------------
        self.digital_on = False
        self.signal_on = False
        self.power_on = False

        # -------------------------------------------------------------
        # Button connections
        # -------------------------------------------------------------
        self.btn_digital.clicked.connect(lambda: self.handle_toggle("digital"))
        self.btn_signal.clicked.connect(lambda: self.handle_toggle("signal"))
        self.btn_power.clicked.connect(lambda: self.handle_toggle("power"))
        self.btn_emergency_stop.clicked.connect(self.stop_all)
        self.btn_send.clicked.connect(self.handle_send)
        self.btn_rest.clicked.connect(lambda: self.set_all_motor_mode("Rest"))
        self.btn_setzero.clicked.connect(lambda: self.set_all_motor_mode("Set Zero"))

        self.update_button_states()

    # -------------------------------------------------------------
    # Insert module widget
    # -------------------------------------------------------------
    def _insert_module(self, groupbox, widget):
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(widget)
        groupbox.setLayout(layout)

    # -------------------------------------------------------------
    # Power control logic
    # -------------------------------------------------------------
    def handle_toggle(self, name):
        if name == "digital":
            if self.power_on:
                return
            self.digital_on = not self.digital_on
            self.update_led("digital", self.digital_on)

        elif name == "signal":
            if not self.signal_on:
                if not self.digital_on:
                    return
            else:
                if self.power_on:
                    return
            self.signal_on = not self.signal_on
            self.update_led("signal", self.signal_on)

        elif name == "power":
            if not self.power_on and not self.signal_on:
                return
            self.power_on = not self.power_on
            self.update_led("power", self.power_on)

        self.power_node.publish_power_command(
            self.digital_on, self.signal_on, self.power_on
        )
        self.update_button_states()

    def update_led(self, name, state):
        led = getattr(self, f"led_{name}")
        color = "limegreen" if state else "gray"
        led.setStyleSheet(f"background-color: {color}; border-radius: 10px;")

    def update_button_states(self):
        self.btn_digital.setEnabled(False)
        self.btn_signal.setEnabled(False)
        self.btn_power.setEnabled(False)

        if self.power_on:
            self.btn_power.setEnabled(True)
            return
        if self.signal_on:
            self.btn_signal.setEnabled(True)
            self.btn_power.setEnabled(True)
            return
        if self.digital_on:
            self.btn_digital.setEnabled(True)
            self.btn_signal.setEnabled(True)
            return
        self.btn_digital.setEnabled(True)

    def stop_all(self):
        self.digital_on = False
        self.signal_on = False
        self.power_on = False
        for n in ["digital", "signal", "power"]:
            self.update_led(n, False)
        self.power_node.publish_power_command(False, False, False)
        self.update_button_states()
        print("Emergency stop: all power off.")

    # -------------------------------------------------------------
    # Motor command handling
    # -------------------------------------------------------------
    def handle_send(self):
        modules = {
            "A": self.module_a,
            "B": self.module_b,
            "C": self.module_c,
            "D": self.module_d,
        }
        self.motor_node.publish_motor_command(modules)

    def set_all_motor_mode(self, mode_name: str):
        """Set all motor comboboxes to given mode and publish motor command."""
        for module in [self.module_a, self.module_b, self.module_c, self.module_d]:
            for motor in ["hip", "steering", "hub"]:
                combo = getattr(module, f"comboBox_{motor}_mode")
                index = combo.findText(mode_name)
                if index >= 0:
                    combo.setCurrentIndex(index)
        self.handle_send()

    # -------------------------------------------------------------
    # Qt custom events (Power and Motor updates)
    # -------------------------------------------------------------
    def customEvent(self, event):
        """Handle custom Qt events for motor and power updates."""
        if isinstance(event, MotorStateUpdateEvent):
            for module_name, joints in event.modules_state.items():
                module_panel = getattr(self, f"module_{module_name.lower()}", None)
                if module_panel:
                    for joint_name, (pos, vel, tor) in joints.items():
                        module_panel.update_motor_state(joint_name, pos, vel, tor)

        elif isinstance(event, VoltageUpdateEvent):
            self.text_voltage_display.setText(f"{event.voltage:.5f} V")

    # -------------------------------------------------------------
    # Cleanup handling (GUI close / Ctrl+C / crash)
    # -------------------------------------------------------------
    def cleanup(self):
        """Safe shutdown for ROS and threads."""
        if getattr(self, "_cleaned", False):
            return
        self._cleaned = True

        print("\n[Panel] Cleaning up resources...")

        time.sleep(0.5) 

        try:
            self.executor.shutdown()
        except Exception as e:
            print(f"[Panel] Executor shutdown error: {e}")

        try:
            self.power_node.destroy_node()
            self.motor_node.destroy_node()
        except Exception as e:
            print(f"[Panel] Node destroy error: {e}")

        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"[Panel] ROS shutdown error: {e}")

        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2)
            print("[Panel] Executor thread joined.")

        print("[Panel] Cleanup complete.")

    def closeEvent(self, event):
        """Triggered when GUI is closed."""
        self.cleanup()
        event.accept()


# -------------------------------------------------------------
# Main entry
# -------------------------------------------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = KilinPanel()

    # -------------------------------------------------------------
    # Handle Ctrl+C (SIGINT)
    # -------------------------------------------------------------
    def handle_sigint(sig, frame):
        print("\n[Panel] Caught Ctrl+C. Shutting down...")
        window.cleanup()
        QtWidgets.QApplication.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    window.show()
    try:
        sys.exit(app.exec_())
    except Exception as e:
        print(f"[Panel] Exception caught: {e}")
        window.cleanup()


if __name__ == "__main__":
    main()
