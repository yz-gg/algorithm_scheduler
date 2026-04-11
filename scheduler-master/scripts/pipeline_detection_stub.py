#!/usr/bin/env python3

import math

import rospy

from scheduler.msg import ImagePoint
from scheduler.msg import PipelineCenterlineObservation


class PipelineDetectionStub(object):
    def __init__(self):
        self.observation_topic = rospy.get_param(
            "~observation_topic", "/inspection/pipeline_observation"
        )
        self.image_width = rospy.get_param("~image_width", 1280)
        self.image_height = rospy.get_param("~image_height", 720)
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 5.0)
        self.detected = rospy.get_param("~detected", True)
        self.confidence = rospy.get_param("~confidence", 0.85)
        self.range_hint_m = rospy.get_param("~range_hint_m", -1.0)
        self.animate = rospy.get_param("~animate", True)
        self.turn_direction = rospy.get_param("~turn_direction", "right")

        self.publisher = rospy.Publisher(
            self.observation_topic, PipelineCenterlineObservation, queue_size=10
        )

    def _point(self, x, y):
        point = ImagePoint()
        point.x = int(round(x))
        point.y = int(round(y))
        return point

    def build_primary_line(self, phase, intersection_x):
        points = []
        x_bias = 0.0
        if self.animate:
            x_bias = 40.0 * math.sin(phase * 0.4)

        start_y = int(self.image_height * 0.05)
        end_y = int(self.image_height * 0.45)
        for y in range(start_y, end_y, 8):
            x = intersection_x + x_bias + 0.03 * (y - end_y)
            points.append(self._point(x, y))
        return points

    def build_secondary_line(self, phase, intersection_x, intersection_y):
        direction = 1.0 if self.turn_direction.lower() == "right" else -1.0
        points = []

        for offset in range(0, 340, 10):
            x = intersection_x + direction * offset
            y = intersection_y + 0.08 * offset
            if self.animate:
                y += 6.0 * math.sin(phase * 0.3)
            if 0 <= x < self.image_width and 0 <= y < self.image_height:
                points.append(self._point(x, y))
        return points

    def build_message(self, phase):
        msg = PipelineCenterlineObservation()
        msg.header.stamp = rospy.Time.now()
        msg.detected = self.detected
        msg.confidence = self.confidence if self.detected else 0.0
        msg.image_width = self.image_width
        msg.image_height = self.image_height
        msg.range_hint_m = self.range_hint_m

        if not self.detected:
            return msg

        intersection_x = self.image_width * 0.55
        intersection_y = self.image_height * 0.78
        if self.animate:
            intersection_y += 80.0 * math.sin(phase * 0.15)

        msg.primary_line_points = self.build_primary_line(phase, intersection_x)
        msg.has_secondary_line = True
        msg.secondary_line_points = self.build_secondary_line(
            phase, intersection_x, intersection_y
        )
        msg.has_intersection = True
        msg.intersection_point = self._point(intersection_x, intersection_y)
        return msg

    def run(self):
        phase = 0.0
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.publisher.publish(self.build_message(phase))
            phase += 0.08
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pipeline_detection_stub")
    PipelineDetectionStub().run()
