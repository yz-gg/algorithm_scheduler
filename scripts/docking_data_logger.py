#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time
from datetime import datetime

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class DockingDataLogger:
    def __init__(self):
        self.enabled = rospy.get_param("~enabled", True)
        self.log_directory = os.path.expanduser(
            rospy.get_param("~log_directory", "~/scheduler_logs/docking")
        )
        self.log_prefix = rospy.get_param("~log_prefix", "docking")
        self.flush_every_n_lines = int(rospy.get_param("~flush_every_n_lines", 20))
        self.parse_json_payloads = rospy.get_param("~parse_json_payloads", True)

        self.blue_light_topic = rospy.get_param("~blue_light_topic", "/docking/blue_light")
        self.dock_pose_topic = rospy.get_param("~dock_pose_topic", "/docking/dock_pose")
        self.apriltag_topic = rospy.get_param("~apriltag_topic", "/docking/apriltag")
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel"
        )

        self.handle = None
        self.line_count = 0
        self.log_path = ""

        if self.enabled:
            self._open_log_file()
            rospy.on_shutdown(self.close)
        else:
            rospy.loginfo("Docking data logger is disabled")

        self.subscribers = [
            rospy.Subscriber(
                self.blue_light_topic,
                String,
                self._string_callback,
                callback_args="vision_blue_light",
                queue_size=100,
            ),
            rospy.Subscriber(
                self.dock_pose_topic,
                String,
                self._string_callback,
                callback_args="vision_dock_pose",
                queue_size=100,
            ),
            rospy.Subscriber(
                self.apriltag_topic,
                String,
                self._string_callback,
                callback_args="vision_apriltag",
                queue_size=100,
            ),
            rospy.Subscriber(
                self.cmd_vel_topic,
                TwistStamped,
                self._cmd_vel_callback,
                queue_size=200,
            ),
        ]

    def _open_log_file(self):
        os.makedirs(self.log_directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "{}_{}.jsonl".format(self.log_prefix, timestamp)
        self.log_path = os.path.join(self.log_directory, filename)
        self.handle = open(self.log_path, "a", encoding="utf-8")
        rospy.loginfo("Docking data logger writing to %s", self.log_path)

    def _base_record(self, event):
        now = rospy.Time.now()
        return {
            "event": event,
            "ros_time": now.to_sec(),
            "wall_time": time.time(),
        }

    def _string_callback(self, msg, event):
        record = self._base_record(event)
        record["topic_payload_raw"] = msg.data

        if self.parse_json_payloads:
            try:
                record["topic_payload"] = json.loads(msg.data)
            except ValueError as exc:
                record["payload_parse_error"] = str(exc)

        self._write(record)

    def _cmd_vel_callback(self, msg):
        record = self._base_record("mavros_cmd_vel")
        record["header_stamp"] = msg.header.stamp.to_sec()
        record["frame_id"] = msg.header.frame_id
        record["linear"] = {
            "x": msg.twist.linear.x,
            "y": msg.twist.linear.y,
            "z": msg.twist.linear.z,
        }
        record["angular"] = {
            "x": msg.twist.angular.x,
            "y": msg.twist.angular.y,
            "z": msg.twist.angular.z,
        }
        self._write(record)

    def _write(self, record):
        if not self.enabled or self.handle is None:
            return

        json.dump(record, self.handle, ensure_ascii=False, separators=(",", ":"))
        self.handle.write("\n")
        self.line_count += 1

        if self.flush_every_n_lines > 0 and self.line_count % self.flush_every_n_lines == 0:
            self.handle.flush()

    def close(self):
        if self.handle is None:
            return

        self.handle.flush()
        self.handle.close()
        self.handle = None


def main():
    rospy.init_node("docking_data_logger", anonymous=False)
    DockingDataLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
