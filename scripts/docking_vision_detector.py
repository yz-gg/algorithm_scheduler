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

try:
    from sensor_msgs.msg import CompressedImage, Image
except ImportError:
    CompressedImage = None
    Image = None


INIT_THRESH = 240
INIT_KERNEL_SIZE = 7
INIT_EPSILON = 0.02

RECT_LENGTH = 1.18
RECT_WIDTH = 0.85
TAG_SIZE = 0.055

DEFAULT_FX = 838.164
DEFAULT_FY = 838.263
DEFAULT_CX = 692.584
DEFAULT_CY = 502.072
DEFAULT_DIST_COEFFS = [0.0278657, -0.0251761, 0.0, 0.0, 0.0]


def nothing(_):
    pass


def get_param(name, default):
    if rospy is not None and rospy.core.is_initialized():
        return rospy.get_param("~" + name, default)
    return default


def should_shutdown():
    return rospy is not None and rospy.core.is_initialized() and rospy.is_shutdown()


def append_json_log(path, data):
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    with open(path, "a", encoding="utf-8") as handle:
        json.dump(data, handle, separators=(",", ":"))
        handle.write("\n")


def publish_json(publisher, data):
    if publisher is None:
        return

    msg = String()
    msg.data = json.dumps(data)
    publisher.publish(msg)


def select_processing_frame(frame, use_left_half):
    if not use_left_half:
        return frame
    _, width = frame.shape[:2]
    if width < 2:
        return frame
    return frame[:, : width // 2]


def publish_processed_image(image_pub, compressed_pub, frame, frame_id="docking_vision"):
    if frame is None:
        return

    header = None
    if image_pub is not None:
        image_msg = Image()
        image_msg.header.stamp = rospy.Time.now()
        image_msg.header.frame_id = frame_id
        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = frame.shape[1] * frame.shape[2]
        image_msg.data = np.ascontiguousarray(frame).tobytes()
        image_pub.publish(image_msg)
        header = image_msg.header

    if compressed_pub is not None:
        ok, encoded = cv2.imencode(".jpg", frame)
        if ok:
            compressed_msg = CompressedImage()
            if header is None:
                compressed_msg.header.stamp = rospy.Time.now()
                compressed_msg.header.frame_id = frame_id
            else:
                compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            compressed_pub.publish(compressed_msg)


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
    if cv2.contourArea(cnt) <= 0.0:
        return None, None

    moments = cv2.moments(cnt)
    if abs(moments["m00"]) > 1e-6:
        center = np.array(
            [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]],
            dtype=np.float32,
        )
    else:
        center = np.mean(cnt.reshape(-1, 2).astype(np.float32), axis=0)

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


def process_light_frame(frame, thresh, kernel_size, epsilon):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    mask_raw = mask.copy()

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=1)
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=1)
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

    corners = sort_corners(approx.reshape(4, 2).astype(np.float32))
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
    return rotation_to_euler(R)


def rotation_to_euler(R):
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


def build_light_payload(light_center, image_shape, camera_matrix, pose_detected=False, euler_deg=None, tvec=None):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    light_detected = light_center is not None
    data = {
        "timestamp": time.time(),
        "detected": bool(pose_detected),
        "light_detected": bool(light_detected),
        "pose_detected": bool(pose_detected),
        "image_width": int(image_shape[1]),
        "image_height": int(image_shape[0]),
        "image_center_px": {"x": float(cx), "y": float(cy)},
    }

    if light_detected:
        yaw_error = math.atan2(float(light_center[0]) - cx, fx)
        heave_error = math.atan2(float(light_center[1]) - cy, fy)
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


def build_apriltag_payload(detected, tag_euler_deg=None, cam_position_m=None):
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


def detect_apriltag(frame, detector, camera_matrix, tag_size, undistort, dist_coeffs):
    if undistort:
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    draw = frame.copy()

    if not tags:
        return build_apriltag_payload(False), draw

    tag = tags[0]
    corners = tag.corners.astype(np.float32)
    half = tag_size / 2.0
    object_pts = np.array(
        [[-half, -half], [half, -half], [half, half], [-half, half]],
        dtype=np.float64,
    )

    H = homography_from_corners(object_pts, corners)
    R, t = decompose_homography(H, camera_matrix)
    R_cam_to_tag = R.T
    t_cam_in_tag = -R_cam_to_tag @ t
    euler_deg = np.degrees(rotation_to_euler(R))
    payload = build_apriltag_payload(True, euler_deg, t_cam_in_tag)

    for pt in corners.astype(int):
        cv2.circle(draw, tuple(pt), 4, (255, 0, 0), -1)
    cv2.putText(
        draw,
        "Tag yaw:{:.1f} Cam:({:.2f},{:.2f},{:.2f})".format(
            euler_deg[2],
            t_cam_in_tag[0, 0],
            t_cam_in_tag[1, 0],
            t_cam_in_tag[2, 0],
        ),
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 0, 0),
        2,
    )
    return payload, draw


def get_camera_matrix(frame_width, frame_height):
    fx = float(get_param("fx", DEFAULT_FX))
    fy = float(get_param("fy", DEFAULT_FY))
    cx = float(get_param("cx", DEFAULT_CX))
    cy = float(get_param("cy", DEFAULT_CY))
    if cx < 0.0:
        cx = frame_width * 0.5
    if cy < 0.0:
        cy = frame_height * 0.5
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def main():
    if rospy is not None:
        rospy.init_node("docking_vision_detector", anonymous=False)

    camera_index = int(get_param("camera_index", 0))
    frame_width = int(get_param("frame_width", 640))
    frame_height = int(get_param("frame_height", 240))
    enable_light = bool(get_param("enable_light", True))
    enable_apriltag = bool(get_param("enable_apriltag", True))
    write_json = bool(get_param("write_json", False))
    publish_topics = bool(get_param("publish_topics", True))
    show_windows = bool(get_param("show_debug_windows", False))
    use_left_half = bool(get_param("use_left_half", False))
    publish_processed_stream = bool(get_param("publish_processed_stream", True))
    publish_compressed_stream = bool(get_param("publish_compressed_stream", True))
    processed_image_topic = get_param("processed_image_topic", "/docking/vision/image")
    processed_image_frame_id = get_param("processed_image_frame_id", "docking_vision")

    light_json_file = get_param("light_json_file", "/home/sodaoh258/pose_data.jsonl")
    apriltag_json_file = get_param("apriltag_json_file", "/home/sodaoh258/apriltag_pose.jsonl")
    blue_light_topic = get_param("blue_light_topic", "/docking/blue_light")
    dock_pose_topic = get_param("dock_pose_topic", "/docking/dock_pose")
    apriltag_topic = get_param("apriltag_topic", "/docking/apriltag")

    thresh_param = int(get_param("threshold", INIT_THRESH))
    kernel_param = int(get_param("kernel_size", INIT_KERNEL_SIZE))
    epsilon_param = float(get_param("epsilon", INIT_EPSILON))
    tag_size = float(get_param("tag_size", TAG_SIZE))
    tag_family = get_param("tag_family", "tag36h11")
    undistort = bool(get_param("undistort", False))
    dist_coeffs = np.array(get_param("dist_coeffs", DEFAULT_DIST_COEFFS), dtype=np.float64)

    light_pub = None
    dock_pub = None
    tag_pub = None
    image_pub = None
    compressed_pub = None
    if publish_topics and rospy is not None and rospy.core.is_initialized():
        light_pub = rospy.Publisher(blue_light_topic, String, queue_size=10)
        dock_pub = rospy.Publisher(dock_pose_topic, String, queue_size=10)
        tag_pub = rospy.Publisher(apriltag_topic, String, queue_size=10)
        if publish_processed_stream:
            if Image is None or CompressedImage is None:
                rospy.logwarn("sensor_msgs is unavailable; disabled processed image stream")
            else:
                image_pub = rospy.Publisher(processed_image_topic, Image, queue_size=1)
                if publish_compressed_stream:
                    compressed_pub = rospy.Publisher(
                        processed_image_topic + "/compressed",
                        CompressedImage,
                        queue_size=1,
                    )

    detector = None
    if enable_apriltag:
        try:
            import pupil_apriltags as apriltag
        except ImportError as exc:
            print("无法导入 pupil_apriltags: {}".format(exc), file=sys.stderr)
            return 1
        detector = apriltag.Detector(families=tag_family)

    cap = cv2.VideoCapture(camera_index)
    if frame_width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    if frame_height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    if not cap.isOpened():
        print("无法打开摄像头{}".format(camera_index), file=sys.stderr)
        return 1

    ok, first_frame = cap.read()
    if not ok:
        print("无法读取摄像头{}图像".format(camera_index), file=sys.stderr)
        cap.release()
        return 1

    first_processing_frame = select_processing_frame(first_frame, use_left_half)
    camera_matrix = get_camera_matrix(
        first_processing_frame.shape[1], first_processing_frame.shape[0]
    )
    if show_windows and enable_light:
        create_trackbars("Light Strip Control")

    print(
        "回坞视觉启动: camera={}, frame={}x{}, processing={}x{}, left_half={}, light={}, apriltag={}, stream={}, logs={}".format(
            camera_index,
            first_frame.shape[1],
            first_frame.shape[0],
            first_processing_frame.shape[1],
            first_processing_frame.shape[0],
            use_left_half,
            enable_light,
            enable_apriltag,
            processed_image_topic if image_pub is not None else "disabled",
            "enabled" if write_json else "disabled",
        )
    )

    half_len = RECT_LENGTH / 2.0
    half_wid = RECT_WIDTH / 2.0
    dock_object_pts = np.array(
        [
            [-half_len, -half_wid, 0],
            [half_len, -half_wid, 0],
            [half_len, half_wid, 0],
            [-half_len, half_wid, 0],
        ],
        dtype=np.float32,
    )

    pending_frame = first_frame
    while not should_shutdown():
        if pending_frame is None:
            ret, frame = cap.read()
            if not ret:
                break
        else:
            frame = pending_frame
            pending_frame = None

        processing_frame = select_processing_frame(frame, use_left_half)
        display = processing_frame.copy()

        if enable_light:
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

            corners, light_center, mask_raw, mask_proc, light_result = process_light_frame(
                processing_frame, thresh, kernel_size, epsilon
            )

            pose_detected = False
            euler_deg = None
            tvec = None
            if corners is not None:
                ok, rvec, solved_tvec = cv2.solvePnP(
                    dock_object_pts,
                    corners,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if ok:
                    pose_detected = True
                    euler_deg = np.degrees(rvec_to_euler(rvec))
                    tvec = solved_tvec

            light_payload = build_light_payload(
                light_center,
                processing_frame.shape,
                camera_matrix,
                pose_detected=pose_detected,
                euler_deg=euler_deg,
                tvec=tvec,
            )
            publish_json(light_pub, light_payload)
            publish_json(dock_pub, light_payload)
            if write_json:
                append_json_log(light_json_file, light_payload)
            display = light_result

        if enable_apriltag and detector is not None:
            tag_payload, tag_display = detect_apriltag(
                processing_frame,
                detector,
                camera_matrix,
                tag_size,
                undistort,
                dist_coeffs,
            )
            publish_json(tag_pub, tag_payload)
            if write_json:
                append_json_log(apriltag_json_file, tag_payload)
            if not enable_light:
                display = tag_display

        publish_processed_image(image_pub, compressed_pub, display, processed_image_frame_id)

        if show_windows:
            cv2.imshow("Docking Vision", display)
            if enable_light:
                cv2.imshow("Light Raw Mask", mask_raw)
                cv2.imshow("Light Processed Mask", mask_proc)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    cap.release()
    if show_windows:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
