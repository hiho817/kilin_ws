from PyQt5 import QtWidgets, uic
from module_panel import Ui_Form
import math

class ModulePanel(QtWidgets.QWidget, Ui_Form):
    def __init__(self, module_name="A"):
        super().__init__()
        self.setupUi(self)
        self.module_name = module_name

        # Initialize default values (all zeros)
        for name in ["hip", "steering", "hub"]:
            for suffix in ["kp", "ki", "kd", "pos_cmd", "vel_cmd", "tor_cmd"]:
                widget = getattr(self, f'lineEdit_{name}_{suffix}', None)
                if (widget):
                    widget.setText("0.0")
        for name in ["hip", "steering", "hub"]:
            combo = getattr(self, f'comboBox_{name}_mode', None)
            if combo:
                idx = combo.findText("Rest")
                if (idx >= 0):
                    combo.setCurrentIndex(idx)
                else:
                    combo.setCurrentIndex(0)

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
        pos_deg = self._safe_float(f"lineEdit_{name}_pos_cmd")
        vel = self._safe_float(f"lineEdit_{name}_vel_cmd")
        tor = self._safe_float(f"lineEdit_{name}_tor_cmd")

        # Convert degree to radian
        pos = math.radians(pos_deg)
        
        # Normalize hip / steering positions to [0, 2pi]
        if (name in ["hip", "steering"]):
            pos = pos % (2 * math.pi)

        return {
            "mode": mode,
            "kp": kp, "ki": ki, "kd": kd,
            "position": pos, "velocity": vel, "torque": tor
        }

    # Update displayed state values (from ROS feedback)
    def update_motor_state(self, name, pos, vel, tor, mode=None, error=None):
        # Convert rad → deg
        pos_deg = math.degrees(pos)
        getattr(self, f"text_{name}_pos_state").setText(f"{pos_deg:.5f}")
        getattr(self, f"text_{name}_vel_state").setText(f"{vel:.5f}")
        getattr(self, f"text_{name}_tor_state").setText(f"{tor:.5f}")

        # Motor mode (optional)
        if mode is not None:
            mode_map = {
                0: "Rest", 1: "Config", 2: "Set Zero",
                3: "HAL Calibrate", 4: "Position Mode",
                5: "Velocity Mode", 6: "Torque Mode"
            }
            getattr(self, f"text_{name}_mode_state").setText(
                mode_map.get(mode, str(mode))
            )

        # ---------- Error Code (new) ----------
        if error is not None:
            error_map = {
                0: "OK",
                1: "TIMEOUT"
            }
            widget = getattr(self, f"text_{name}_error")
            widget.setText(error_map.get(error, str(error)))

            # Smaller font for error text
            font = widget.font()
            font.setPointSize(8)  # 8pt small font
            widget.setFont(font)
            if error == 0:
                widget.setStyleSheet("")
            else:
                widget.setStyleSheet("background-color: #ffb3b3; border-radius: 3px;")


    # Helper: safely convert text to float
    def _safe_float(self, widget_name):
        try:
            text = getattr(self, widget_name).text().strip()
            return float(text) if text else 0.0
        except Exception:
            return 0.0
