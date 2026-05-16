#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import pupil_apriltags as apriltag

try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    rospy = None
    String = None


FX = 1115.5075
FY = 1116.0352
CX = 692.3729
CY = 503.0312
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
D = np.array([0.428537, 0.157768, 0, 0, 0], dtype=np.float64)
TAG_SIZE = 0.055


def get_param(name, default):
    if rospy is not None and rospy.core.is_initialized():
        return rospy.get_param("~" + name, default)
    return default


def should_shutdown():
    return rospy is not None and rospy.core.is_initialized() and rospy.is_shutdown()


def publish_json(publisher, data):
    if publisher is None:
        return

    msg = String()
    msg.data = json.dumps(data)
    publisher.publish(msg)


def append_json_log(path, data):
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    with open(path, "a", encoding="utf-8") as handle:
        json.dump(data, handle, separators=(",", ":"))
        handle.write("\n")


def inverse_matrix(K_matrix):
    a, b, c = K_matrix[0]
    d, e, f = K_matrix[1]
    g, h, i = K_matrix[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return np.array(
        [
            [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
        ],
        dtype=np.float64,
    )


def homography_from_corners(obj_pts, img_pts):
    A = []
    for i in range(4):
        X, Y = obj_pts[i]
        u, v = img_pts[i]
        A.append([-X, -Y, -1, 0, 0, 0, u * X, u * Y, u])
        A.append([0, 0, 0, -X, -Y, -1, v * X, v * Y, v])
    A = np.array(A, dtype=np.float64)
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


def decompose_homography(H, K_matrix):
    invK = inverse_matrix(K_matrix)
    RT = invK @ H
    r1 = RT[:, 0]
    r2 = RT[:, 1]
    t = RT[:, 2]
    scale = np.linalg.norm(r1)
    r1 = r1 / scale
    r2 = r2 / scale
    t = t / scale
    r3 = np.cross(r1, r2)
    R = np.column_stack((r1, r2, r3))
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = U @ Vt
    return R, t.reshape(3, 1)


def R_to_euler(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    return np.array([roll, pitch, yaw])


def build_payload(detected, tag_euler_deg=None, cam_position_m=None):
    data = {"timestamp": time.time(), "detected": bool(detected)}
    if detected and tag_euler_deg is not None and cam_position_m is not None:
        data["tag_euler_deg"] = {
            "roll": float(tag_euler_deg[0]),
            "pitch": float(tag_euler_deg[1]),
            "yaw": float(tag_euler_deg[2]),
        }
        data["camera_position_in_tag_m"] = {
            "x": float(cam_position_m[0, 0]),
            "y": float(cam_position_m[1, 0]),
            "z": float(cam_position_m[2, 0]),
        }
    return data


def main():
    if rospy is not None:
        rospy.init_node("docking_apriltag_detector", anonymous=False)

    camera_index = int(get_param("camera_index", 0))
    json_output_file = get_param("json_output_file", "/home/sodaoh258/apriltag_pose.jsonl")
    write_json = bool(get_param("write_json", False))
    publish_topics = bool(get_param("publish_topics", True))
    apriltag_topic = get_param("apriltag_topic", "/docking/apriltag")
    tag_size = float(get_param("tag_size", TAG_SIZE))
    tag_family = get_param("tag_family", "tag36h11")
    show_windows = bool(get_param("show_debug_windows", False))
    undistort = bool(get_param("undistort", False))

    apriltag_pub = None
    if publish_topics and rospy is not None and rospy.core.is_initialized():
        apriltag_pub = rospy.Publisher(apriltag_topic, String, queue_size=10)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("无法打开摄像头{}".format(camera_index), file=sys.stderr)
        return 1

    at_detector = apriltag.Detector(families=tag_family)
    print("AprilTag 检测启动，JSON 日志: {}".format(json_output_file if write_json else "disabled"))

    while not should_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        if undistort:
            frame = cv2.undistort(frame, K, D)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray)
        draw = frame.copy()

        if len(tags) > 0:
            tag = tags[0]
            corners = tag.corners.astype(np.float32)
            half = tag_size / 2.0
            object_pts = np.array(
                [[-half, -half], [half, -half], [half, half], [-half, half]],
                dtype=np.float64,
            )

            H = homography_from_corners(object_pts, corners)
            R, t = decompose_homography(H, K)
            R_cam_to_tag = R.T
            t_cam_in_tag = -R_cam_to_tag @ t
            euler_deg = np.degrees(R_to_euler(R))
            payload = build_payload(True, euler_deg, t_cam_in_tag)
            if write_json:
                append_json_log(json_output_file, payload)
            publish_json(apriltag_pub, payload)

            if show_windows:
                for pt in corners.astype(int):
                    cv2.circle(draw, tuple(pt), 4, (255, 0, 0), -1)
                cv2.putText(
                    draw,
                    "Roll:{:.1f} Pitch:{:.1f} Yaw:{:.1f}".format(
                        euler_deg[0], euler_deg[1], euler_deg[2]
                    ),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    draw,
                    "Cam in Tag: ({:.2f}, {:.2f}, {:.2f}) m".format(
                        t_cam_in_tag[0, 0], t_cam_in_tag[1, 0], t_cam_in_tag[2, 0]
                    ),
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
        else:
            payload = build_payload(False)
            if write_json:
                append_json_log(json_output_file, payload)
            publish_json(apriltag_pub, payload)
            if show_windows:
                cv2.putText(
                    draw,
                    "No tag detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )

        if show_windows:
            cv2.imshow("AprilTag Result", draw)
            if cv2.waitKey(25) & 0xFF == 27:
                break

    cap.release()
    if show_windows:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
