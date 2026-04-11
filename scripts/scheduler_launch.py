#!/usr/bin/env python3

import os
import queue
import threading

import rospy
import roslaunch
import rospkg
import yaml

from scheduler.srv import ControlModule, ControlModuleResponse

""" 
# 配置文件路径
# 配置yaml配置文件路径, 需要此文件位于的scripts文件夹与yaml文件所在的config文件夹在同一目录
# 配置launch文件路径, 需要此文件位于的scripts文件夹与launch文件所在的launch文件夹在同一目录
"""
yaml_path = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "launch_registry.yaml")
)


class LaunchManager(object):
    def __init__(self):
        self.registry = {}
        self.lock = threading.Lock()
        self.rospack = rospkg.RosPack()
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.build_registry()

    def build_registry(self):
        config_file = yaml_path
        with open(config_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
            data = config.get("modules", {})

        for module_name, module_info in data.items():
            package = module_info.get("package", "")
            launch_name = module_info.get("launch", "")
            launch_arguments = self._normalize_launch_arguments(
                module_info.get("arguments", {})
            )
            self.registry[module_name] = {
                "package": package,
                "launch": launch_name,
                "launch_file": self._resolve_launch_file(package, launch_name),
                "launch_arguments": launch_arguments,
                "description": module_info.get("description", ""),
                "state": "STOPPED",
                "parent": None,
            }

    def _normalize_launch_arguments(self, arguments):
        if arguments is None:
            return []

        if isinstance(arguments, dict):
            normalized = []
            for key, value in arguments.items():
                if isinstance(value, bool):
                    value = "true" if value else "false"
                normalized.append("{}:={}".format(str(key), str(value)))
            return normalized

        if isinstance(arguments, list):
            return [str(item) for item in arguments]

        if isinstance(arguments, str):
            return [arguments] if arguments.strip() else []

        raise TypeError("unsupported launch arguments type: {}".format(type(arguments)))

    def _resolve_launch_file(self, package, launch_name):
        package_path = self.rospack.get_path(package)
        launch_file = os.path.join(package_path, "launch", launch_name)
        if not os.path.isfile(launch_file):
            raise FileNotFoundError("Launch file not found: {}".format(launch_file))
        return launch_file

    def start_module(self, name):
        with self.lock:
            entry = self.registry.get(name)
            if entry is None:
                return False, "UNKNOWN", "module not registered"

            if entry["state"] == "RUNNING":
                return True, entry["state"], "module already running"

            if entry["state"] == "STARTING":
                return False, entry["state"], "module is starting"

            try:
                entry["state"] = "STARTING"
                roslaunch_files = [
                    (entry["launch_file"], entry["launch_arguments"])
                ]
                parent = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files)
                parent.start(auto_terminate=False)
                entry["parent"] = parent
                entry["state"] = "RUNNING"
                rospy.loginfo(
                    "Module [%s] started with launch args: %s",
                    name,
                    entry["launch_arguments"],
                )
                return True, entry["state"], "module started"
            except Exception as exc:
                entry["state"] = "ERROR"
                entry["parent"] = None
                rospy.logerr("Start module [%s] failed: %s", name, str(exc))
                return False, entry["state"], str(exc)

    def stop_module(self, name):
        with self.lock:
            entry = self.registry.get(name)
            if entry is None:
                return False, "UNKNOWN", "module not registered"

            if entry["state"] == "STOPPED":
                return True, entry["state"], "module already stopped"

            if entry["state"] != "RUNNING":
                return False, entry["state"], "module is not running"

            try:
                entry["state"] = "STOPPING"
                if entry["parent"] is not None:
                    entry["parent"].shutdown()
                entry["parent"] = None
                entry["state"] = "STOPPED"
                rospy.loginfo("Module [%s] stopped", name)
                return True, entry["state"], "module stopped"
            except Exception as exc:
                entry["state"] = "ERROR"
                rospy.logerr("Stop module [%s] failed: %s", name, str(exc))
                return False, entry["state"], str(exc)

    def get_module_status(self, name):
        if name not in self.registry:
            return None
        return self.registry[name]["state"]

    def shutdown_all(self):
        for module_name in list(self.registry.keys()):
            success, _, _ = self.stop_module(module_name)
            if not success and self.get_module_status(module_name) != "STOPPED":
                rospy.logwarn("Failed to stop module [%s] during shutdown", module_name)


class LaunchManageNode:
    def __init__(self):
        rospy.init_node("launch_manager_node")
        self.manager = LaunchManager()
        self.test_module_name = rospy.get_param("~test_autostart_module", "")
        self.test_start_delay_sec = rospy.get_param("~test_start_delay_sec", 1.0)
        self.command_queue = queue.Queue()
        self.control_service = rospy.Service(
            "~control_module", ControlModule, self.handle_control_module
        )
        rospy.on_shutdown(self.manager.shutdown_all)
        self._schedule_test_start()
        rospy.loginfo("Launch manager node is ready")

    def _schedule_test_start(self):
        if not self.test_module_name:
            return

        rospy.loginfo(
            "Standalone test is enabled, module [%s] will start in %.2f seconds",
            self.test_module_name,
            self.test_start_delay_sec,
        )
        timer = threading.Timer(self.test_start_delay_sec, self._enqueue_test_start)
        timer.daemon = True
        timer.start()

    def _enqueue_test_start(self):
        self.command_queue.put(
            {
                "module_name": self.test_module_name,
                "command": "start",
                "response_event": None,
                "response": None,
                "origin": "standalone_test",
            }
        )

    def _execute_command(self, module_name, command):
        if command == "start":
            return self.manager.start_module(module_name)
        if command == "stop":
            return self.manager.stop_module(module_name)
        if command == "status":
            state = self.manager.get_module_status(module_name)
            if state is None:
                return False, "UNKNOWN", "module not registered"
            return True, state, "module status queried"
        return False, "UNKNOWN", "unsupported command: {}".format(command)

    def _handle_command(self, command_item):
        module_name = command_item["module_name"]
        command = command_item["command"]
        success, state, message = self._execute_command(module_name, command)

        response_event = command_item["response_event"]
        if response_event is not None:
            command_item["response"] = (success, state, message)
            response_event.set()
            return

        if success:
            rospy.loginfo(
                "Standalone test started module [%s], state=%s, message=%s",
                module_name,
                state,
                message,
            )
            return

        rospy.logerr(
            "Standalone test failed to start module [%s], state=%s, message=%s",
            module_name,
            state,
            message,
        )

    def handle_control_module(self, req):
        command = req.command.strip().lower()
        module_name = req.module_name.strip()

        if not module_name:
            return ControlModuleResponse(
                success=False,
                state="UNKNOWN",
                message="module_name is empty",
            )

        response_event = threading.Event()
        command_item = {
            "module_name": module_name,
            "command": command,
            "response_event": response_event,
            "response": None,
            "origin": "service",
        }
        self.command_queue.put(command_item)

        while not rospy.is_shutdown():
            if response_event.wait(timeout=0.1):
                success, state, message = command_item["response"]
                return ControlModuleResponse(
                    success=success,
                    state=state,
                    message=message,
                )

        return ControlModuleResponse(
            success=False,
            state="ERROR",
            message="launch manager is shutting down",
        )

    def spin(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            try:
                command_item = self.command_queue.get_nowait()
                self._handle_command(command_item)
            except queue.Empty:
                pass
            rate.sleep()


if __name__ == "__main__":
    node = LaunchManageNode()
    node.spin()
