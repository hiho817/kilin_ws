from PyQt5 import QtWidgets, uic
from module_panel import Ui_Form

class ModulePanel(QtWidgets.QWidget, Ui_Form):
    def __init__(self, module_name="A"):
        super().__init__()
        self.setupUi(self)
        self.module_name = module_name

    # Retrieve all motor commands from this module
    def get_leg_cmd(self):
        return {
            "hip": self._read_motor("hip"),
            "steering": self._read_motor("steering"),
            "hub": self._read_motor("hub")
        }

    # Read a single motor's parameters (mode, PID, command values)
    def _read_motor(self, name):
        # Mode selection
        mode_widget = getattr(self, f"comboBox_{name}_mode")
        mode = mode_widget.currentText()

        # PID gains
        kp = self._safe_float(f"lineEdit_{name}_kp")
        ki = self._safe_float(f"lineEdit_{name}_ki")
        kd = self._safe_float(f"lineEdit_{name}_kd")

        # Command inputs (position, velocity, torque)
        pos = self._safe_float(f"lineEdit_{name}_pos_cmd")
        vel = self._safe_float(f"lineEdit_{name}_vel_cmd")
        tor = self._safe_float(f"lineEdit_{name}_tor_cmd")

        return {
            "mode": mode,
            "kp": kp, "ki": ki, "kd": kd,
            "position": pos, "velocity": vel, "torque": tor
        }

    # Update displayed state values (from ROS feedback)
    def update_motor_state(self, name, pos, vel, tor):
        getattr(self, f"text_{name}_pos_state").setText(f"{pos:.2f}")
        getattr(self, f"text_{name}_vel_state").setText(f"{vel:.2f}")
        getattr(self, f"text_{name}_tor_state").setText(f"{tor:.2f}")

    # Helper: safely convert text to float
    def _safe_float(self, widget_name):
        try:
            text = getattr(self, widget_name).text().strip()
            return float(text) if text else 0.0
        except Exception:
            return 0.0
