#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time
from datetime import datetime

import rospy
from geometry_msgs.msg import TwistStamped
from scheduler.msg import PipelineCenterlineObservation


class PipelineDataLogger:
    def __init__(self):
        self.enabled = rospy.get_param("~enabled", True)
        self.log_directory = os.path.expanduser(
            rospy.get_param("~log_directory", "~/scheduler_logs/pipeline_inspection")
        )
        self.log_prefix = rospy.get_param("~log_prefix", "pipeline_inspection")
        self.flush_every_n_lines = int(rospy.get_param("~flush_every_n_lines", 20))

        self.observation_topic = rospy.get_param(
            "~observation_topic", "/inspection/pipeline_observation"
        )
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
            rospy.loginfo("Pipeline data logger is disabled")

        self.subscribers = [
            rospy.Subscriber(
                self.observation_topic,
                PipelineCenterlineObservation,
                self._observation_callback,
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
        rospy.loginfo("Pipeline data logger writing to %s", self.log_path)

    def _base_record(self, event):
        now = rospy.Time.now()
        return {
            "event": event,
            "ros_time": now.to_sec(),
            "wall_time": time.time(),
        }

    @staticmethod
    def _point_to_dict(point):
        return {
            "x": int(point.x),
            "y": int(point.y),
        }

    def _observation_callback(self, msg):
        record = self._base_record("pipeline_observation")
        record["header_stamp"] = msg.header.stamp.to_sec()
        record["frame_id"] = msg.header.frame_id
        record["detected"] = bool(msg.detected)
        record["confidence"] = float(msg.confidence)
        record["image_width"] = int(msg.image_width)
        record["image_height"] = int(msg.image_height)
        record["primary_line_points"] = [
            self._point_to_dict(point) for point in msg.primary_line_points
        ]
        record["has_secondary_line"] = bool(msg.has_secondary_line)
        record["secondary_line_points"] = [
            self._point_to_dict(point) for point in msg.secondary_line_points
        ]
        record["has_intersection"] = bool(msg.has_intersection)
        record["intersection_point"] = self._point_to_dict(msg.intersection_point)
        record["range_hint_m"] = float(msg.range_hint_m)
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
    rospy.init_node("pipeline_data_logger", anonymous=False)
    PipelineDataLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
