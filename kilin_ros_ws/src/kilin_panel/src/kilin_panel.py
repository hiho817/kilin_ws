from PyQt5 import QtWidgets, QtCore
from mainwindow import Ui_MainWindow
from module_widget import ModulePanel
from power_node import PowerNode, PowerStateUpdateEvent
from motor_node import MotorNode, MotorStateUpdateEvent
from kilin_msgs.msg import MotorCmdStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import signal
import sys
import time
import math


class KilinPanel(QtWidgets.QMainWindow, Ui_MainWindow):
    """Main GUI for Kilin control system."""

    # Qt signal to safely update UI from ROS callbacks (runs in GUI thread)
    motor_cmd_signal = QtCore.pyqtSignal(object)

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
        # Additional ROS interfaces for command forwarding
        # -------------------------------------------------------------
        self.node_ui = rclpy.create_node("kilin_panel_ui")
        # Forwarded command topic to bridge/sbRIO
        self.pub_motor_cmd = self.node_ui.create_publisher(
            MotorCmdStamped, "/motor/command", 10
        )
        # Raw command from cmd_converter (joystick + kinematics)
        self.sub_converter = self.node_ui.create_subscription(
            MotorCmdStamped, "/kilin/motor_cmd_raw", self.converter_callback, 10
        )
        self.executor.add_node(self.node_ui)

        # Connect Qt signal to UI update slot (runs in GUI thread)
        self.motor_cmd_signal.connect(self.update_ui_from_motorcmd)

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
        # Control mode selection (UI / Manual)
        # -------------------------------------------------------------
        self.comboBox_input.currentTextChanged.connect(self.on_mode_changed)
        self.current_mode = self.comboBox_input.currentText()  # "UI" or "Manual"
        # Set initial editability based on current mode from UI file
        self.set_manual_editable(self.current_mode == "UI")

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
        self.btn_brake.clicked.connect(self.apply_brake_preset)

        self.update_button_states()

        # -------------------------------------------------------------
        # Motor safety states
        # -------------------------------------------------------------
        self.motor_error_latched = False

        # Persistent motor error filter (per-motor consecutive count)
        self.motor_error_threshold = 10  # consecutive non-zero error codes
        self.motor_error_count = {}      # key: (module, joint), value: int
        self.motor_error_last_code = {}  # key: (module, joint), value: last error code

        # Debug log control (avoid spam)
        self._motor_err_log_last_ns = 0
        self._MOTOR_ERR_LOG_PERIOD_NS = int(1e9 * 1.0)  # 1 Hz max debug logs

        self.node_ui.get_logger().info(f"KilinPanel initialized (mode={self.current_mode})")

    # -------------------------------------------------------------
    # Mode switching and converter forwarding
    # -------------------------------------------------------------
    def on_mode_changed(self, mode: str):
        """Triggered when control mode ComboBox changes."""
        self.current_mode = mode
        self.node_ui.get_logger().info(f"[UI] Control mode switched to: {mode}")

        # Safety: send zero command on mode switch to stop motors immediately
        zero_cmd = MotorCmdStamped()
        now = self.node_ui.get_clock().now().to_msg()
        # Header compatibility: use .stamp when available
        try:
            zero_cmd.header.stamp = now
        except Exception:
            # Fallback for custom header layouts (e.g. time.sec / time.nanosec)
            pass
        zero_cmd.header.frame_id = f"mode_switch_{mode.lower()}"
        self.pub_motor_cmd.publish(zero_cmd)
        self.node_ui.get_logger().warn(f"[UI] Sent zero command on mode switch → {mode}")

        # Toggle input editability (Manual: read-only and gray; UI: editable and white)
        self.set_manual_editable(mode == "UI")

    def converter_callback(self, msg: MotorCmdStamped):
        """
        ROS callback for /kilin/motor_cmd_raw.
        In Manual mode:
          - Forward to /motor/command
          - Emit signal to update UI in GUI thread
        """
        if self.current_mode == "Manual":
            # Forward command to bridge/sbRIO
            self.pub_motor_cmd.publish(msg)

            # Emit Qt signal so UI update runs in GUI thread (thread-safe)
            self.motor_cmd_signal.emit(msg)

            self.node_ui.get_logger().debug(
                "Forwarded motor_cmd_raw → /motor/command and scheduled UI update"
            )

    # -------------------------------------------------------------
    # Update displayed state in Manual mode (pos / vel / tor only)
    # -------------------------------------------------------------
    def update_ui_from_motorcmd(self, msg: MotorCmdStamped):
        """
        Update UI fields to reflect incoming motor commands in Manual mode.
        Only updates pos / vel / tor state fields to keep UI lightweight.
        This function runs in the GUI thread via motor_cmd_signal.
        """
        try:
            # Guard: only update UI in Manual mode
            if self.current_mode != "Manual":
                return

            modules = {
                "A": msg.module_a,
                "B": msg.module_b,
                "C": msg.module_c,
                "D": msg.module_d,
            }

            for name, leg in modules.items():
                panel = getattr(self, f"module_{name.lower()}", None)
                if panel is None:
                    continue

                # --- Hip ---
                hip_pos = math.degrees(getattr(leg.hip, "position", 0.0))
                panel.text_hip_pos_state.setText(f"{hip_pos:.3f}")
                panel.text_hip_vel_state.setText(f"{getattr(leg.hip, 'velocity', 0.0):.3f}")
                panel.text_hip_tor_state.setText(f"{getattr(leg.hip, 'torque', 0.0):.3f}")

                # --- Steering ---
                steering_pos = math.degrees(getattr(leg.steering, "position", 0.0))
                panel.text_steering_pos_state.setText(f"{steering_pos:.3f}")
                panel.text_steering_vel_state.setText(f"{getattr(leg.steering, 'velocity', 0.0):.3f}")
                panel.text_steering_tor_state.setText(f"{getattr(leg.steering, 'torque', 0.0):.3f}")

                # --- Hub ---
                hub_pos = math.degrees(getattr(leg.hub, "position", 0.0))
                panel.text_hub_pos_state.setText(f"{hub_pos:.3f}")
                panel.text_hub_vel_state.setText(f"{getattr(leg.hub, 'velocity', 0.0):.3f}")
                panel.text_hub_tor_state.setText(f"{getattr(leg.hub, 'torque', 0.0):.3f}")

        except Exception as e:
            self.node_ui.get_logger().error(f"UI update error: {e}")

    def set_manual_editable(self, enabled: bool):
        """
        Enable/disable all input fields in ModulePanels.
        When disabled (Manual mode), fields are read-only and gray.
        """
        panels = [self.module_a, self.module_b, self.module_c, self.module_d]
        for panel in panels:
            for name in ["hip", "steering", "hub"]:
                for suffix in ["kp", "ki", "kd", "pos_cmd", "vel_cmd", "tor_cmd"]:
                    widget = getattr(panel, f"lineEdit_{name}_{suffix}", None)
                    if widget:
                        widget.setReadOnly(not enabled)
                        color = "#ffffff" if enabled else "#e0e0e0"
                        widget.setStyleSheet(f"background-color: {color};")

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
    def handle_toggle(self, name: str):
        """Handle power/digital/signal toggle button logic."""
        if name == "digital":
            if self.power_on:
                return
            self.digital_on = not self.digital_on
        elif name == "signal":
            if not self.signal_on:
                if not self.digital_on:
                    return
            else:
                if self.power_on:
                    return
            self.signal_on = not self.signal_on

        elif name == "power":
            if not self.power_on and not self.signal_on:
                return
            self.power_on = not self.power_on

        self.power_node.publish_power_command(
            self.digital_on, self.signal_on, self.power_on
        )
        self.update_button_states()

    def update_led(self, name: str, state: bool):
        """Update LED indicator color."""
        led = getattr(self, f"led_{name}")
        color = "limegreen" if state else "gray"
        led.setStyleSheet(f"background-color: {color}; border-radius: 10px;")

    def update_button_states(self):
        """Enable/disable power control buttons based on current state."""
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
        """Emergency stop: turn off all power and LEDs."""
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
        """Triggered when Send button is pressed in UI mode."""
        if self.current_mode == "UI":
            modules = {
                "A": self.module_a,
                "B": self.module_b,
                "C": self.module_c,
                "D": self.module_d,
            }
            self.motor_node.publish_motor_command(modules)
            self.node_ui.get_logger().info("[UI] Published manual /motor/command")
        else:
            self.node_ui.get_logger().warn(
                "[UI] Manual send disabled in Manual mode (converter active)"
            )

    def set_all_motor_mode(self, mode_name: str):
        """Set all motor comboboxes to the given mode and publish motor command."""
        for module in [self.module_a, self.module_b, self.module_c, self.module_d]:
            for motor in ["hip", "steering", "hub"]:
                combo = getattr(module, f"comboBox_{motor}_mode")
                index = combo.findText(mode_name)
                if index >= 0:
                    combo.setCurrentIndex(index)
        self.handle_send()

    def apply_brake_preset(self):
        """
        Brake preset:
        - Hub: Brake
        - Steering: Position mode, angle = 0 deg
        - Hip: Rest
        Then publish once via handle_send().
        """
        modules = [self.module_a, self.module_b, self.module_c, self.module_d]

        for module in modules:
            # -------------------------
            # Hip -> Rest
            # -------------------------
            hip_mode = getattr(module, "comboBox_hip_mode", None)
            if hip_mode:
                idx = hip_mode.findText("Rest")
                if idx >= 0:
                    hip_mode.setCurrentIndex(idx)

            # -------------------------
            # Steering -> Position, pos_cmd = 0 deg
            # -------------------------
            steering_mode = getattr(module, "comboBox_steering_mode", None)
            if steering_mode:
                idx = steering_mode.findText("Position")
                if idx < 0:
                    for cand in ["POSITION", "Pos", "Position Mode"]:
                        idx = steering_mode.findText(cand)
                        if idx >= 0:
                            break
                if idx >= 0:
                    steering_mode.setCurrentIndex(idx)

            steering_pos_cmd = getattr(module, "lineEdit_steering_pos_cmd", None)
            if steering_pos_cmd:
                steering_pos_cmd.setText("0.0")

            # -------------------------
            # Hub -> Brake
            # -------------------------
            hub_mode = getattr(module, "comboBox_hub_mode", None)
            if hub_mode:
                idx = hub_mode.findText("Brake")
                if idx < 0:
                    for cand in ["BRAKE", "Brk", "Brake Mode"]:
                        idx = hub_mode.findText(cand)
                        if idx >= 0:
                            break
                if idx >= 0:
                    hub_mode.setCurrentIndex(idx)

        self.handle_send()
        self.node_ui.get_logger().info("[UI] Brake preset applied: hub=Brake, steering=Position@0deg, hip=Rest")



    # -------------------------------------------------------------
    # Motor error safety handler
    # -------------------------------------------------------------
    def handle_motor_error_event(self):
        """Safety fallback: triggered whenever any motor reports a confirmed persistent error."""
        if self.motor_error_latched:
            return  # Already handled
        self.motor_error_latched = True

        self.node_ui.get_logger().error("[SAFETY] Persistent motor error detected! Switching to safe mode.")

        # 1. If currently in Manual mode, force switch back to UI mode
        if self.current_mode == "Manual":
            self.node_ui.get_logger().warn("[SAFETY] Forced switch to UI control mode.")
            self.comboBox_input.setCurrentText("UI")

        # 2. Set all motors in UI to Rest mode
        for module in [self.module_a, self.module_b, self.module_c, self.module_d]:
            for motor in ["hip", "steering", "hub"]:
                combo = getattr(module, f"comboBox_{motor}_mode")
                index = combo.findText("Rest")
                if index >= 0:
                    combo.setCurrentIndex(index)

        # 3. Publish the Rest command once
        self.handle_send()

        self.node_ui.get_logger().warn("[SAFETY] All motors forced to Rest mode due to persistent motor error.")

    # -------------------------------------------------------------
    # Qt custom events (Power and Motor updates)
    # -------------------------------------------------------------
    def customEvent(self, event):
        """Handle custom Qt events for motor and power updates."""
        if isinstance(event, MotorStateUpdateEvent):

            motor_error_confirmed = False  # confirmed persistent error

            # Throttled debug logging
            now_ns = self.node_ui.get_clock().now().nanoseconds
            allow_debug_log = (now_ns - self._motor_err_log_last_ns) >= self._MOTOR_ERR_LOG_PERIOD_NS
            log_lines = []

            for module_name, joints in event.modules_state.items():
                module_panel = getattr(self, f"module_{module_name.lower()}", None)
                if module_panel:
                    for joint_name, (pos, vel, tor, mode, error) in joints.items():
                        module_panel.update_motor_state(
                            joint_name, pos, vel, tor, mode, error
                        )

                        # -------------------------------------------------
                        # Persistent error filter (per-motor consecutive count)
                        # -------------------------------------------------
                        key = (module_name, joint_name)

                        if error == 0:
                            # Reset counter on OK
                            if self.motor_error_count.get(key, 0) != 0 and allow_debug_log:
                                log_lines.append(f"{module_name}/{joint_name}: recovered (cnt -> 0)")
                            self.motor_error_count[key] = 0
                            self.motor_error_last_code[key] = 0
                        else:
                            # Count consecutive non-zero errors
                            prev = self.motor_error_count.get(key, 0)
                            self.motor_error_count[key] = prev + 1

                            # Track last error code (for debug)
                            last_code = self.motor_error_last_code.get(key, None)
                            self.motor_error_last_code[key] = error

                            if allow_debug_log:
                                # Log only when (a) first error, (b) code changes, or (c) near threshold
                                if prev == 0 or (last_code is not None and error != last_code) or (self.motor_error_count[key] in [3, 5, 8, self.motor_error_threshold]):
                                    log_lines.append(
                                        f"{module_name}/{joint_name}: error={error} cnt={self.motor_error_count[key]}/{self.motor_error_threshold}"
                                    )

                            # Confirm error only if it persists N times
                            if self.motor_error_count[key] >= self.motor_error_threshold:
                                motor_error_confirmed = True

            if allow_debug_log and log_lines:
                self._motor_err_log_last_ns = now_ns
                self.node_ui.get_logger().debug("[MotorErrorMonitor] " + " | ".join(log_lines))

            # Trigger safety routine only when error is confirmed
            if motor_error_confirmed:
                self.handle_motor_error_event()
            else:
                # If no motor is currently confirmed in persistent error,
                # allow future error episodes to trigger again.
                self.motor_error_latched = False

        elif isinstance(event, PowerStateUpdateEvent):
            # LEDs
            self.update_led("digital", event.power_state["digital"])
            self.update_led("signal",  event.power_state["signal"])
            self.update_led("power",   event.power_state["power"])

            # Voltage display (use v_0)
            self.text_voltage_display.setText(f"{event.power_state['voltages'][0]:.5f} V")

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
            self.node_ui.destroy_node()
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
        """Triggered when GUI window is closed."""
        self.cleanup()
        event.accept()


# -------------------------------------------------------------
# Main entry
# -------------------------------------------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = KilinPanel()

    # Handle Ctrl+C (SIGINT) to shutdown cleanly
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
