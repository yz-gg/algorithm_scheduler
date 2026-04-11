#!/usr/bin/env python3

import json
import os
import shlex
import subprocess
import threading

import rospy

from scheduler.msg import ImagePoint
from scheduler.msg import PipelineCenterlineObservation


class ExternalPipelineAlgorithmBridge(object):
    def __init__(self):
        self.observation_topic = rospy.get_param(
            "~observation_topic", "/inspection/pipeline_observation"
        )
        self.python_executable = rospy.get_param("~python_executable", "python3")
        self.algorithm_script = rospy.get_param("~algorithm_script", "")
        self.algorithm_working_directory = rospy.get_param(
            "~algorithm_working_directory", ""
        )
        self.algorithm_args = rospy.get_param("~algorithm_args", "")
        self.restart_on_exit = rospy.get_param("~restart_on_exit", False)
        self.restart_delay_sec = rospy.get_param("~restart_delay_sec", 1.0)

        self.publisher = rospy.Publisher(
            self.observation_topic, PipelineCenterlineObservation, queue_size=10
        )

        self.process = None
        self.stderr_thread = None

    def _parse_algorithm_args(self):
        if not self.algorithm_args:
            return []

        try:
            return shlex.split(self.algorithm_args, posix=(os.name != "nt"))
        except ValueError as exc:
            rospy.logwarn("Failed to parse algorithm_args: %s", str(exc))
            return self.algorithm_args.split()

    def _resolve_working_directory(self):
        if self.algorithm_working_directory:
            return self.algorithm_working_directory

        if not self.algorithm_script:
            return None

        return os.path.dirname(os.path.abspath(self.algorithm_script))

    def _start_process(self):
        if not self.algorithm_script:
            rospy.logfatal("Parameter ~algorithm_script is empty")
            raise RuntimeError("algorithm_script is empty")

        command = [self.python_executable, self.algorithm_script]
        command.extend(self._parse_algorithm_args())

        working_directory = self._resolve_working_directory()
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        environment["PYTHONIOENCODING"] = "utf-8"
        self.process = subprocess.Popen(
            command,
            cwd=working_directory if working_directory else None,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            universal_newlines=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )

        self.stderr_thread = threading.Thread(target=self._pump_stderr)
        self.stderr_thread.daemon = True
        self.stderr_thread.start()

        rospy.loginfo("Started external algorithm: %s", " ".join(command))

    def _stop_process(self):
        if self.process is None:
            return

        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()

        self.process = None

    def _pump_stderr(self):
        if self.process is None or self.process.stderr is None:
            return

        for line in self.process.stderr:
            if rospy.is_shutdown():
                return
            text = line.strip()
            if text:
                rospy.logwarn("[external_algorithm] %s", text)

    def _to_point(self, value):
        point = ImagePoint()

        if isinstance(value, dict):
            point.x = int(round(float(value.get("x", 0.0))))
            point.y = int(round(float(value.get("y", 0.0))))
            return point

        if isinstance(value, (list, tuple)) and len(value) >= 2:
            point.x = int(round(float(value[0])))
            point.y = int(round(float(value[1])))
            return point

        raise ValueError("point must be {'x': ..., 'y': ...} or [x, y]")

    def _to_points(self, values):
        return [self._to_point(value) for value in values]

    def _build_message(self, payload):
        msg = PipelineCenterlineObservation()
        msg.header.stamp = rospy.Time.now()
        msg.detected = bool(payload.get("detected", True))
        msg.confidence = float(
            payload.get("confidence", 1.0 if msg.detected else 0.0)
        )
        msg.image_width = int(payload.get("image_width", 0))
        msg.image_height = int(payload.get("image_height", 0))
        msg.range_hint_m = float(payload.get("range_hint_m", -1.0))

        msg.primary_line_points = self._to_points(
            payload.get("primary_line_points", [])
        )

        secondary_points = payload.get("secondary_line_points", [])
        msg.has_secondary_line = bool(
            payload.get("has_secondary_line", len(secondary_points) > 0)
        )
        msg.secondary_line_points = self._to_points(secondary_points)

        intersection = payload.get("intersection_point")
        msg.has_intersection = bool(
            payload.get("has_intersection", intersection is not None)
        )
        if intersection is not None:
            msg.intersection_point = self._to_point(intersection)

        return msg

    def run(self):
        try:
            self._start_process()
        except Exception as exc:
            rospy.logfatal("Failed to start external algorithm: %s", str(exc))
            return

        while not rospy.is_shutdown():
            if self.process is None or self.process.stdout is None:
                break

            line = self.process.stdout.readline()
            if not line:
                return_code = self.process.poll()
                if return_code is None:
                    rospy.sleep(0.05)
                    continue

                rospy.logwarn("External algorithm exited with code %s", return_code)
                self._stop_process()

                if not self.restart_on_exit:
                    rospy.signal_shutdown("external algorithm exited")
                    return

                rospy.sleep(self.restart_delay_sec)
                try:
                    self._start_process()
                except Exception as exc:
                    rospy.logerr("Failed to restart external algorithm: %s", str(exc))
                    rospy.sleep(self.restart_delay_sec)
                continue

            text = line.strip()
            if not text:
                continue

            if not text.startswith("{"):
                rospy.loginfo_throttle(
                    10.0, "Ignoring non-JSON stdout from external algorithm"
                )
                continue

            try:
                payload = json.loads(text)
                if not isinstance(payload, dict):
                    rospy.logwarn("External algorithm JSON payload must be an object")
                    continue
                self.publisher.publish(self._build_message(payload))
            except Exception as exc:
                rospy.logwarn("Failed to parse external algorithm output: %s", str(exc))

        self._stop_process()


if __name__ == "__main__":
    rospy.init_node("external_pipeline_algorithm_bridge")
    bridge = ExternalPipelineAlgorithmBridge()
    rospy.on_shutdown(bridge._stop_process)
    bridge.run()
