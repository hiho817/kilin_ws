"""Run one reproducible real-Kilin terrain trial from its local YAML file."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

import yaml


REQUIRED_CONTROLLER_KEYS = (
    "armed",
    "mode",
    "use_speed_command",
    "speed_m_s",
    "run_duration_s",
    "hard_motion_limit_s",
    "use_odometry",
    "odometry_relative_origin",
    "use_terrain_window",
    "terrain_window_timeout_s",
    "vicon_trigger",
    "debug_publish",
)


def _require_mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a YAML mapping")
    return value


def load_trial_config(trial_dir: Path) -> dict[str, Any]:
    """Load and validate the intentionally small trial-runner schema."""
    config_path = trial_dir / "trial_config.yaml"
    if not config_path.is_file():
        raise ValueError(f"Missing trial config: {config_path}")
    with config_path.open(encoding="utf-8") as stream:
        config = _require_mapping(yaml.safe_load(stream), str(config_path))
    for key in ("recording", "fastlio", "terrain_mapper", "controller"):
        config[key] = _require_mapping(config.get(key), f"{config_path}:{key}")
    console_logging = config.get("console_logging", {"enabled": True, "directory": "console"})
    config["console_logging"] = _require_mapping(console_logging, f"{config_path}:console_logging")
    console_directory = config["console_logging"].get("directory", "console")
    if not isinstance(console_directory, str) or not console_directory:
        raise ValueError("console_logging.directory must be a non-empty trial-local path")
    directory_path = Path(console_directory)
    if directory_path.is_absolute() or ".." in directory_path.parts:
        raise ValueError("console_logging.directory must stay inside the trial directory")

    recording = config["recording"]
    mode = recording.get("mode")
    if mode not in ("all", "selected"):
        raise ValueError("recording.mode must be 'all' or 'selected'")
    if mode == "selected":
        topics = recording.get("topics")
        if not isinstance(topics, list) or not topics or not all(
            isinstance(topic, str) and topic.startswith("/") for topic in topics
        ):
            raise ValueError(
                "recording.topics must be a non-empty list of absolute ROS topics "
                "when recording.mode is 'selected'"
            )

    controller = config["controller"]
    missing = [key for key in REQUIRED_CONTROLLER_KEYS if key not in controller]
    if missing:
        raise ValueError(
            "controller is missing required keys: " + ", ".join(missing)
        )
    if "startup_wait_s" not in controller:
        raise ValueError("controller is missing required key: startup_wait_s")
    for key in ("terrain_profile", "hip_pid_profile"):
        relative = controller.get(key)
        if not isinstance(relative, str) or not relative:
            raise ValueError(f"controller.{key} must name a trial-local YAML file")
        if not (trial_dir / relative).is_file():
            raise ValueError(
                f"controller.{key} does not exist in this trial: {trial_dir / relative}"
            )
    if config["fastlio"].get("enabled", True):
        relative = config["fastlio"].get("config")
        if not isinstance(relative, str) or not (trial_dir / relative).is_file():
            raise ValueError("fastlio.config must name an existing trial-local YAML file")
    mapper = config["terrain_mapper"]
    if bool(mapper.get("enabled", False)):
        relative = mapper.get("profile")
        if not isinstance(relative, str) or not (trial_dir / relative).is_file():
            raise ValueError(
                "terrain_mapper.profile must name an existing trial-local ROS parameter YAML file"
            )
    return config


def _ros_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class TrialRunner:
    def __init__(self, trial_dir: Path, config: dict[str, Any], allow_armed: bool):
        self.trial_dir = trial_dir
        self.config = config
        self.allow_armed = allow_armed
        console_logging = config["console_logging"]
        self.log_dir: Path | None = None
        if bool(console_logging.get("enabled", True)):
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.log_dir = trial_dir / str(console_logging.get("directory", "console")) / stamp
            self.log_dir.mkdir(parents=True, exist_ok=False)
        self.processes: list[tuple[str, subprocess.Popen[Any], Any]] = []
        self._runner_start_monotonic = time.monotonic()
        self.environment = os.environ.copy()
        self.environment["PYTHONNOUSERSITE"] = "1"

    def _runner_log(self, message: str) -> None:
        stamped = f"{datetime.now().isoformat(timespec='seconds')} {message}\n"
        if self.log_dir is not None:
            with (self.log_dir / "runner.log").open("a", encoding="utf-8") as stream:
                stream.write(stamped)
        print(message, flush=True)

    def _log_location(self, name: str) -> str:
        if self.log_dir is None:
            return "the invoking terminal (console_logging.enabled=false)"
        return str(self.log_dir / f"{name}.log")

    def _start(self, name: str, command: list[str]) -> None:
        stream = None if self.log_dir is None else (self.log_dir / f"{name}.log").open("w", encoding="utf-8")
        self._runner_log(f"Starting {name}; console log: {self._log_location(name)}")
        self._runner_log("Command: " + " ".join(command))
        process = subprocess.Popen(
            command,
            stdout=stream,
            stderr=subprocess.STDOUT if stream is not None else None,
            env=self.environment,
            start_new_session=True,
        )
        self.processes.append((name, process, stream))

    def _wait_until_start_offset(self, component: str, offset_s: float) -> None:
        """Wait until a component's configured offset from runner invocation."""
        if offset_s < 0.0:
            raise ValueError(f"{component}.startup_wait_s must be non-negative")
        remaining_s = offset_s - (time.monotonic() - self._runner_start_monotonic)
        if remaining_s > 0.0:
            self._runner_log(
                f"Waiting {remaining_s:.1f} s to start {component} at its "
                f"configured {offset_s:.1f} s offset"
            )
            time.sleep(remaining_s)

    def _start_fastlio(self) -> None:
        config = self.config["fastlio"]
        if not bool(config.get("enabled", True)):
            return
        self._wait_until_start_offset("fastlio", float(config.get("startup_wait_s", 0.0)))
        command = [
            "ros2", "launch",
            str(config.get("package", "kilin_fastlio_bringup")),
            str(config.get("launch_file", "mid360s_fastlio.launch.py")),
            f"fastlio_config:={self.trial_dir / config['config']}",
        ]
        self._start("fastlio", command)

    def _wait_for_first_message(
        self, topic: str, timeout_s: float, log_name: str, component: str
    ) -> None:
        """Refuse to start control when a required upstream topic is absent."""
        log_path = None if self.log_dir is None else self.log_dir / f"{log_name}.log"
        command = ["ros2", "topic", "echo", topic, "--once"]
        self._runner_log(
            f"Waiting up to {timeout_s:.1f} s for {component} input on {topic}; "
            f"readiness log: {self._log_location(log_name)}"
        )
        try:
            if log_path is not None:
                with log_path.open("w", encoding="utf-8") as stream:
                    result = subprocess.run(
                        command, stdout=stream, stderr=subprocess.STDOUT,
                        env=self.environment, timeout=timeout_s, check=False,
                    )
            else:
                result = subprocess.run(
                    command, env=self.environment, timeout=timeout_s, check=False,
                )
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                f"{component} readiness failed: no message arrived on {topic} within "
                f"{timeout_s:.1f} s. The controller was not started. Read "
                f"{self._log_location(log_name)}."
            ) from error
        if result.returncode != 0:
            raise RuntimeError(
                f"{component} readiness command exited with status {result.returncode}; "
                f"the controller was not started. Read {self._log_location(log_name)}."
            )

    def _start_mapper(self) -> None:
        config = self.config["terrain_mapper"]
        if not bool(config.get("enabled", False)):
            return
        self._wait_until_start_offset("terrain_mapper", float(config.get("startup_wait_s", 0.0)))
        command = [
            "ros2", "run", "kilin_local_terrain_mapping", "local_terrain_window",
            "--ros-args",
            "--params-file", str(self.trial_dir / config["profile"]),
        ]
        self._start("terrain_mapper", command)

    def _wait_for_required_terrain(self) -> None:
        if not bool(self.config["controller"]["use_terrain_window"]):
            return
        mapper = self.config["terrain_mapper"]
        topic = str(mapper.get("required_terrain_topic", "/kilin/terrain/local_window"))
        timeout_s = float(mapper.get("required_terrain_wait_s", 5.0))
        self._wait_for_first_message(topic, timeout_s, "terrain_mapper_readiness", "terrain mapper")

    def _wait_for_required_odometry(self) -> None:
        if not bool(self.config["controller"]["use_odometry"]):
            return
        fastlio = self.config["fastlio"]
        topic = str(fastlio.get("required_odometry_topic", "/kilin/fastlio/odometry"))
        timeout_s = float(fastlio.get("required_odometry_wait_s", 8.0))
        self._wait_for_first_message(topic, timeout_s, "fastlio_readiness", "FAST-LIO")

    def _start_bag(self) -> None:
        config = self.config["recording"]
        if not bool(config.get("enabled", True)):
            self._runner_log("ROS bag recording disabled by recording.enabled=false")
            return
        self._wait_until_start_offset("recording", float(config.get("startup_wait_s", 0.0)))
        output = self.trial_dir / str(config.get("output_directory", "bag"))
        if output.exists():
            raise RuntimeError(
                f"Refusing to overwrite existing bag directory: {output}. "
                "Create a new trial directory or choose a new output_directory."
            )
        command = ["ros2", "bag", "record", "-o", str(output)]
        if config["mode"] == "all":
            command.append("-a")
        else:
            command.extend(config["topics"])
        self._start("rosbag", command)

    def _controller_command(self) -> list[str]:
        controller = self.config["controller"]
        if bool(controller["armed"]) and not self.allow_armed:
            raise RuntimeError(
                "Trial config requests armed=true. Re-run with --allow-armed only after "
                "checking the trial-local YAMLs, ROS topics, and RViz."
            )
        command = [
            "ros2", "launch", "kilin_known_terrain_controller",
            "real_kilin_known_ramp.launch.py",
        ]
        for key in REQUIRED_CONTROLLER_KEYS:
            command.append(f"{key}:={_ros_value(controller[key])}")
        command.extend(
            [
                f"terrain_profile:={self.trial_dir / controller['terrain_profile']}",
                f"hip_pid_profile:={self.trial_dir / controller['hip_pid_profile']}",
                f"vicon_trigger_test:={_ros_value(controller.get('vicon_trigger_test', False))}",
                "vicon_trigger_test_duration_s:="
                + _ros_value(controller.get("vicon_trigger_test_duration_s", 3.0)),
            ]
        )
        return command

    def _start_controller_and_wait(self) -> None:
        self._wait_until_start_offset(
            "controller", float(self.config["controller"]["startup_wait_s"])
        )
        self._wait_for_required_odometry()
        self._wait_for_required_terrain()
        self._start("controller", self._controller_command())
        _, process, _ = self.processes[-1]
        returncode = process.wait()
        if returncode != 0:
            raise RuntimeError(
                f"Controller exited with status {returncode}; read {self._log_location('controller')}"
            )

    def stop(self) -> None:
        for name, process, stream in reversed(self.processes):
            if process.poll() is None:
                self._runner_log(f"Stopping {name}")
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    self._runner_log(f"Force-stopping {name}")
                    os.killpg(process.pid, signal.SIGTERM)
            if stream is not None:
                stream.close()

    def run(self) -> None:
        self._runner_log(f"Trial directory: {self.trial_dir}")
        self._runner_log(
            "Console logs: " + (str(self.log_dir) if self.log_dir is not None else "disabled")
        )
        self._start_fastlio()
        self._start_mapper()
        self._start_bag()
        self._start_controller_and_wait()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Launch one trial from trial_config.yaml and retain process console logs."
    )
    parser.add_argument("--trial-dir", required=True, type=Path)
    parser.add_argument(
        "--allow-armed",
        action="store_true",
        help="Required when controller.armed is true in trial_config.yaml.",
    )
    args = parser.parse_args(argv)
    try:
        config = load_trial_config(args.trial_dir)
        runner = TrialRunner(args.trial_dir, config, args.allow_armed)
        try:
            runner.run()
        finally:
            runner.stop()
    except (RuntimeError, ValueError, OSError) as error:
        print(f"TRIAL RUNNER ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
