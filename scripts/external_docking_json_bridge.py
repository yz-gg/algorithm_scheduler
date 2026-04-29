#!/usr/bin/env python3

import json
import os

import rospy
from std_msgs.msg import String


class JsonFileSource(object):
    def __init__(self, path, publishers, label):
        self.path = path
        self.publishers = publishers
        self.label = label
        self.last_mtime = None
        self.last_size = None

    def poll(self):
        if not self.path:
            return

        try:
            stat = os.stat(self.path)
        except OSError:
            rospy.logwarn_throttle(
                5.0, "Waiting for %s JSON file: %s", self.label, self.path
            )
            return

        signature = (stat.st_mtime, stat.st_size)
        if signature == (self.last_mtime, self.last_size):
            return

        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                text = handle.read().strip()
            if not text:
                return
            json.loads(text)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "Failed to read %s JSON file %s: %s", self.label, self.path, exc
            )
            return

        self.last_mtime, self.last_size = signature
        msg = String()
        msg.data = text
        for publisher in self.publishers:
            publisher.publish(msg)


class ExternalDockingJsonBridge(object):
    def __init__(self):
        blue_light_topic = rospy.get_param(
            "~blue_light_topic", "/docking/blue_light"
        )
        dock_pose_topic = rospy.get_param("~dock_pose_topic", "/docking/dock_pose")
        apriltag_topic = rospy.get_param("~apriltag_topic", "/docking/apriltag")

        self.poll_rate_hz = rospy.get_param("~poll_rate_hz", 20.0)
        light_json_file = rospy.get_param("~light_json_file", "pose_data.json")
        apriltag_json_file = rospy.get_param(
            "~apriltag_json_file", "apriltag_pose.json"
        )

        self.blue_light_pub = rospy.Publisher(
            blue_light_topic, String, queue_size=10
        )
        self.dock_pose_pub = rospy.Publisher(dock_pose_topic, String, queue_size=10)
        self.apriltag_pub = rospy.Publisher(apriltag_topic, String, queue_size=10)

        self.sources = [
            JsonFileSource(
                light_json_file,
                [self.blue_light_pub, self.dock_pose_pub],
                "light strip pose",
            ),
            JsonFileSource(apriltag_json_file, [self.apriltag_pub], "apriltag pose"),
        ]

    def run(self):
        rate = rospy.Rate(max(float(self.poll_rate_hz), 1.0))
        while not rospy.is_shutdown():
            for source in self.sources:
                source.poll()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("external_docking_json_bridge")
    ExternalDockingJsonBridge().run()
