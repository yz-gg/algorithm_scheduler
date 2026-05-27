#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import sys
import time

import cv2
import numpy as np

try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    rospy = None
    String = None


INIT_THRESH = 240
INIT_KERNEL_SIZE = 7
INIT_EPSILON = 0.02

RECT_LENGTH = 1.18
RECT_WIDTH = 0.85

FX = 838.164
FY = 838.263
CX = 692.584
CY = 502.072
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST_COEFFS = np.array([0.0278657, -0.0251761, 0, 0, 0], dtype=np.float64)


def nothing(_):
    pass


def get_param(name, default):
    if rospy is not None and rospy.core.is_initialized():
        return rospy.get_param("~" + name, default)
    return default


def should_shutdown():
    return rospy is not None and rospy.core.is_initialized() and rospy.is_shutdown()


def publish_json(publishers, data):
    if not publishers:
        return

    msg = String()
    msg.data = json.dumps(data)
    for publisher in publishers:
        publisher.publish(msg)


def create_trackbars(win):
    cv2.namedWindow(win)
    cv2.createTrackbar("Thresh", win, INIT_THRESH, 255, nothing)
    cv2.createTrackbar("Kernel", win, INIT_KERNEL_SIZE, 21, nothing)
    cv2.createTrackbar("Epsilon(x1000)", win, int(INIT_EPSILON * 1000), 100, nothing)


def find_light_candidate(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None

    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area <= 0.0:
        return None, None

    moments = cv2.moments(cnt)
    if abs(moments["m00"]) > 1e-6:
        center = np.array(
            [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]],
            dtype=np.float32,
        )
    else:
        points = cnt.reshape(-1, 2).astype(np.float32)
        center = np.mean(points, axis=0)

    return cnt, center


def sort_corners(corners):
    center = np.mean(corners, axis=0)
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    sorted_pts = corners[np.argsort(angles)]
    sum_xy = sorted_pts[:, 0] + sorted_pts[:, 1]
    diff_xy = sorted_pts[:, 0] - sorted_pts[:, 1]
    tl = sorted_pts[np.argmin(sum_xy)]
    br = sorted_pts[np.argmax(sum_xy)]
    bl = sorted_pts[np.argmin(diff_xy)]
    tr = sorted_pts[np.argmax(diff_xy)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def process_frame(frame, thresh, kernel_size, epsilon):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    mask_raw = mask.copy()

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=1)
    erode_kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, erode_kernel, iterations=1)
    mask_processed = mask.copy()

    cnt, light_center = find_light_candidate(mask)
    result = frame.copy()
    if cnt is None:
        return None, None, mask_raw, mask_processed, result

    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon * peri, True)
    if len(approx) != 4:
        cv2.circle(result, tuple(np.round(light_center).astype(int)), 5, (0, 255, 255), -1)
        return None, light_center, mask_raw, mask_processed, result

    corners = approx.reshape(4, 2).astype(np.float32)
    corners = sort_corners(corners)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    light_center = np.mean(corners, axis=0)

    pts_int = corners.astype(np.int32)
    cv2.polylines(result, [pts_int], True, (0, 0, 0), 4)
    cv2.polylines(result, [pts_int], True, (0, 255, 0), 2)
    for pt in pts_int:
        cv2.circle(result, tuple(pt), 5, (0, 0, 255), -1)
    cv2.circle(result, tuple(np.round(light_center).astype(int)), 5, (0, 255, 255), -1)

    return corners, light_center, mask_raw, mask_processed, result


def rvec_to_euler(rvec):
    R, _ = cv2.Rodrigues(rvec)
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


def compute_angle_error(light_center):
    yaw_error = math.atan2(float(light_center[0]) - CX, FX)
    heave_error = math.atan2(float(light_center[1]) - CY, FY)
    return yaw_error, heave_error


def append_json_log(path, data):
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    with open(path, "a", encoding="utf-8") as handle:
        json.dump(data, handle, separators=(",", ":"))
        handle.write("\n")


def build_payload(light_center, image_shape, pose_detected=False, euler_deg=None, tvec=None):
    light_detected = light_center is not None
    data = {
        "timestamp": time.time(),
        "detected": bool(pose_detected),
        "light_detected": bool(light_detected),
        "pose_detected": bool(pose_detected),
        "image_width": int(image_shape[1]),
        "image_height": int(image_shape[0]),
        "image_center_px": {"x": float(CX), "y": float(CY)},
    }

    if light_detected:
        yaw_error, heave_error = compute_angle_error(light_center)
        data["light_center_px"] = {
            "x": float(light_center[0]),
            "y": float(light_center[1]),
        }
        data["yaw_error_rad"] = float(yaw_error)
        data["heave_error_rad"] = float(heave_error)
        data["yaw_error_deg"] = float(math.degrees(yaw_error))
        data["heave_error_deg"] = float(math.degrees(heave_error))
        data["angle_error_rad"] = {
            "yaw": float(yaw_error),
            "pitch": float(heave_error),
        }
        data["angle_error_deg"] = {
            "yaw": float(math.degrees(yaw_error)),
            "pitch": float(math.degrees(heave_error)),
        }

    if pose_detected and euler_deg is not None and tvec is not None:
        data["euler_deg"] = {
            "roll": float(euler_deg[0]),
            "pitch": float(euler_deg[1]),
            "yaw": float(euler_deg[2]),
        }
        data["position_m"] = {
            "x": float(tvec[0, 0]),
            "y": float(tvec[1, 0]),
            "z": float(tvec[2, 0]),
        }

    return data


def main():
    if rospy is not None:
        rospy.init_node("docking_light_strip_detector", anonymous=False)

    camera_index = int(get_param("camera_index", 1))
    json_output_file = get_param("json_output_file", "/home/sodaoh258/pose_data.jsonl")
    write_json = bool(get_param("write_json", False))
    publish_topics = bool(get_param("publish_topics", True))
    blue_light_topic = get_param("blue_light_topic", "/docking/blue_light")
    dock_pose_topic = get_param("dock_pose_topic", "/docking/dock_pose")
    show_windows = bool(get_param("show_debug_windows", False))
    thresh_param = int(get_param("threshold", INIT_THRESH))
    kernel_param = int(get_param("kernel_size", INIT_KERNEL_SIZE))
    epsilon_param = float(get_param("epsilon", INIT_EPSILON))

    publishers = []
    if publish_topics and rospy is not None and rospy.core.is_initialized():
        publishers = [
            rospy.Publisher(blue_light_topic, String, queue_size=10),
            rospy.Publisher(dock_pose_topic, String, queue_size=10),
        ]

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("无法打开摄像头{}".format(camera_index), file=sys.stderr)
        return 1

    if show_windows:
        create_trackbars("Light Strip Control")

    print("灯带检测启动，JSON 日志: {}".format(json_output_file if write_json else "disabled"))

    half_len = RECT_LENGTH / 2.0
    half_wid = RECT_WIDTH / 2.0
    object_pts = np.array(
        [
            [-half_len, -half_wid, 0],
            [half_len, -half_wid, 0],
            [half_len, half_wid, 0],
            [-half_len, half_wid, 0],
        ],
        dtype=np.float32,
    )

    while not should_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        if show_windows:
            thresh = cv2.getTrackbarPos("Thresh", "Light Strip Control")
            kernel_size = cv2.getTrackbarPos("Kernel", "Light Strip Control")
            eps_val = cv2.getTrackbarPos("Epsilon(x1000)", "Light Strip Control")
            epsilon = max(0.001, eps_val / 1000.0)
        else:
            thresh = thresh_param
            kernel_size = kernel_param
            epsilon = epsilon_param

        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel_size = max(1, kernel_size)

        corners, light_center, mask_raw, mask_proc, result = process_frame(
            frame, thresh, kernel_size, epsilon
        )

        pose_detected = False
        euler_deg = None
        tvec = None
        if corners is not None:
            ok, rvec, solved_tvec = cv2.solvePnP(
                object_pts, corners, K, DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if ok:
                pose_detected = True
                euler_deg = np.degrees(rvec_to_euler(rvec))
                tvec = solved_tvec

        payload = build_payload(
            light_center, frame.shape, pose_detected=pose_detected, euler_deg=euler_deg, tvec=tvec
        )
        if write_json:
            append_json_log(json_output_file, payload)
        publish_json(publishers, payload)

        if show_windows:
            if light_center is None:
                cv2.putText(
                    result,
                    "No light detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )
            else:
                cv2.putText(
                    result,
                    "Yaw err: {:.2f} deg  Heave err: {:.2f} deg".format(
                        payload["yaw_error_deg"], payload["heave_error_deg"]
                    ),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )
                if pose_detected:
                    cv2.putText(
                        result,
                        "Pose yaw: {:.1f}  Pos: ({:.2f}, {:.2f}, {:.2f}) m".format(
                            euler_deg[2], tvec[0, 0], tvec[1, 0], tvec[2, 0]
                        ),
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

            cv2.imshow("Light Original", frame)
            cv2.imshow("Light Raw Mask", mask_raw)
            cv2.imshow("Light Processed Mask", mask_proc)
            cv2.imshow("Light Result", result)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if show_windows:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
