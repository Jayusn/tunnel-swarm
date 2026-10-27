#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from collections import deque
from copy import deepcopy

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401  Registers PointStamped transforms.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import yaml


class ThermalD435FusionNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        default_config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config", "thermal_d435_fusion.yaml"
        )
        self.config_path = rospy.get_param("~config_path", default_config_path)
        self.config = self.load_config(self.config_path)

        self.thermal_target_detected_topic = self.get_param(
            "thermal_target_detected_topic", "/UAV0/thermal/target_detected"
        )
        self.thermal_target_pixel_topic = self.get_param(
            "thermal_target_pixel_topic", "/UAV0/thermal/target_pixel"
        )
        self.thermal_candidate_detected_topic = self.get_param(
            "thermal_candidate_detected_topic", "/UAV0/thermal/target_candidate_detected"
        )
        self.thermal_candidate_pixel_topic = self.get_param(
            "thermal_candidate_pixel_topic", "/UAV0/thermal/target_candidate_pixel"
        )
        self.thermal_candidate_status_topic = self.get_param(
            "thermal_candidate_status_topic", "/UAV0/thermal/target_candidate_status"
        )
        self.thermal_candidate_camera_point_topic = self.get_param(
            "thermal_candidate_camera_point_topic",
            "/UAV0/thermal/target_candidate_camera_point",
        )
        self.report_candidate_topic = self.get_param(
            "report_candidate_topic",
            "",
        )
        self.d435_camera_info_topic = self.get_param(
            "d435_camera_info_topic", "/camera/color/camera_info"
        )
        self.d435_depth_topic = self.get_param(
            "d435_depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.d435_color_topic = self.get_param(
            "d435_color_topic", "/camera/color/image_raw"
        )
        self.d435_debug_image_topic = self.get_param(
            "d435_debug_image_topic", "/UAV0/thermal/d435_debug_image"
        )
        self.d435_evidence_image_topic = self.get_param(
            "d435_evidence_image_topic", self.d435_debug_image_topic
        )

        self.homography = np.array(
            self.get_param(
                "thermal_to_d435_homography",
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            ),
            dtype=np.float64,
        )
        if self.homography.shape != (3, 3):
            rospy.logwarn("Invalid homography shape %s, using identity", self.homography.shape)
            self.homography = np.eye(3, dtype=np.float64)

        self.depth_scale = float(self.get_param("depth_scale", 0.001))
        self.depth_search_radius = int(self.get_param("depth_search_radius", 5))
        self.min_valid_depth = float(self.get_param("min_valid_depth", 0.2))
        self.max_valid_depth = float(self.get_param("max_valid_depth", 5.0))
        self.target_frame = str(self.get_param("target_frame", "odom"))
        self.d435_optical_frame = str(
            self.get_param("d435_optical_frame", "camera_color_optical_frame")
        )
        self.use_tf = bool(self.get_param("use_tf", True))
        self.thermal_ground_filter_enabled = bool(
            self.get_param("thermal_ground_filter_enabled", False)
        )
        self.thermal_min_height = float(
            self.get_param("thermal_min_height_m", -0.08)
        )
        self.publish_rate = float(self.get_param("publish_rate", 20))
        self.debug = bool(self.get_param("debug", True))
        self.d435_exposure_warmup_seconds = max(
            0.0, float(self.get_param("d435_exposure_warmup_seconds", 5.0))
        )
        cache_size = max(1, int(self.get_param("d435_image_cache_size", 10)))
        self.depth_frames = deque(maxlen=cache_size)
        self.color_frames = deque(maxlen=cache_size)
        self.fusion_delay_s = max(0.0, float(self.get_param("fusion_delay_s", 0.0)))
        image_buff_size = int(self.get_param("d435_image_buff_size", 16777216))
        # Retain thermal events while the corresponding RealSense frames arrive.
        # Only metadata is queued; image buffers remain in the two frame caches.
        self.pending_candidate_statuses = deque(
            maxlen=max(100, int(math.ceil(self.fusion_delay_s * max(self.publish_rate, 20))) + 20)
        )
        self.fusion_candidate_status = None
        self.fusion_candidate_status_version = 0

        self.target_detected = False
        self.thermal_pixel_msg = None
        self.candidate_target_detected = False
        self.candidate_thermal_pixel_msg = None
        self.candidate_thermal_bbox = None
        self.candidate_status = None
        self.candidate_status_version = 0
        self.published_candidate_status_version = 0
        self.camera_info = None
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp = rospy.Time(0)
        self.color_image = None
        self.color_header = None
        self.first_color_wall_time = None
        self.d435_exposure_warmup_finished = (
            self.d435_exposure_warmup_seconds <= 0.0
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.d435_pixel_pub = rospy.Publisher(
            "/UAV0/thermal/target_d435_pixel", PointStamped, queue_size=1
        )
        self.camera_point_pub = rospy.Publisher(
            "/UAV0/thermal/target_camera_point", PointStamped, queue_size=1
        )
        self.candidate_camera_point_pub = rospy.Publisher(
            self.thermal_candidate_camera_point_topic, PointStamped, queue_size=1
        )
        self.report_candidate_pub = None
        if self.report_candidate_topic:
            self.report_candidate_pub = rospy.Publisher(
                self.report_candidate_topic, String, queue_size=5
            )
        self.channel_position_pub = rospy.Publisher(
            "/UAV0/thermal/target_channel_position", PointStamped, queue_size=1
        )
        self.fusion_valid_pub = rospy.Publisher("/UAV0/thermal/fusion_valid", Bool, queue_size=1)
        self.d435_debug_image_pub = rospy.Publisher(
            self.d435_debug_image_topic, Image, queue_size=1
        )
        self.d435_evidence_image_pub = rospy.Publisher(
            self.d435_evidence_image_topic, Image, queue_size=1
        )

        rospy.Subscriber(self.thermal_target_detected_topic, Bool, self.detected_callback)
        rospy.Subscriber(self.thermal_target_pixel_topic, rospy.AnyMsg, self.pixel_callback)
        rospy.Subscriber(
            self.thermal_candidate_detected_topic,
            Bool,
            self.candidate_detected_callback,
        )
        rospy.Subscriber(
            self.thermal_candidate_pixel_topic,
            rospy.AnyMsg,
            self.candidate_pixel_callback,
        )
        rospy.Subscriber(
            self.thermal_candidate_status_topic,
            String,
            self.candidate_status_callback,
        )
        rospy.Subscriber(self.d435_camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(
            self.d435_depth_topic, Image, self.depth_callback,
            queue_size=1, buff_size=image_buff_size,
        )
        rospy.Subscriber(
            self.d435_color_topic, Image, self.color_callback,
            queue_size=1, buff_size=image_buff_size,
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_callback)

    def load_config(self, path):
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
        except Exception as exc:
            rospy.logwarn("Failed to load fusion config %s: %s", path, exc)
            return {}
        return loaded if isinstance(loaded, dict) else {}

    def get_param(self, name, default):
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        if rospy.has_param(name):
            return rospy.get_param(name)
        return self.config.get(name, default)

    def detected_callback(self, msg):
        with self.lock:
            self.target_detected = bool(msg.data)

    def pixel_callback(self, msg):
        self._pixel_callback(msg, candidate=False)

    def candidate_detected_callback(self, msg):
        with self.lock:
            self.candidate_target_detected = bool(msg.data)

    def candidate_pixel_callback(self, msg):
        self._pixel_callback(msg, candidate=True)

    def candidate_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            bbox = status.get("bbox")
            if bbox is not None:
                if len(bbox) != 4:
                    raise ValueError("bbox must contain x, y, width, height")
                bbox = tuple(float(value) for value in bbox)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid thermal candidate bbox: %s", exc)
            return

        with self.lock:
            self.candidate_thermal_bbox = bbox
            self.candidate_status = status
            self.candidate_status_version += 1
            if self.fusion_delay_s > 0.0:
                try:
                    event_time = float(status["stamp"])
                except (TypeError, ValueError, KeyError):
                    event_time = rospy.Time.now().to_sec()
                self.pending_candidate_statuses.append(
                    (event_time, self.candidate_status_version, status)
                )

    def fusion_status_snapshot(self, now):
        """Called under self.lock; choose the newest event whose delay has elapsed.

        Keep the last selected event for display/retry if no new one is ready.
        This retains the existing latest-event policy without accumulating work
        when the timer runs slower than the detector. No callback sleeps here.
        """
        if self.fusion_delay_s <= 0.0:
            return self.candidate_status, self.candidate_status_version
        cutoff = now.to_sec() - self.fusion_delay_s
        while self.pending_candidate_statuses:
            event_time, version, status = self.pending_candidate_statuses[0]
            if event_time > cutoff:
                break
            self.pending_candidate_statuses.popleft()
            self.fusion_candidate_status = status
            self.fusion_candidate_status_version = version
        return self.fusion_candidate_status, self.fusion_candidate_status_version

    def _pixel_callback(self, msg, candidate):
        msg_type = msg._connection_header.get("type", "")
        try:
            if msg_type == "geometry_msgs/PointStamped":
                parsed = PointStamped()
                parsed.deserialize(msg._buff)
                point_msg = parsed
            elif msg_type == "geometry_msgs/Point":
                parsed = Point()
                parsed.deserialize(msg._buff)
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "thermal_camera"
                point_msg.point = parsed
            else:
                rospy.logwarn_throttle(2.0, "Unsupported thermal target pixel type: %s", msg_type)
                return
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse thermal target pixel: %s", exc)
            return

        with self.lock:
            if candidate:
                self.candidate_thermal_pixel_msg = point_msg
            else:
                self.thermal_pixel_msg = point_msg

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert depth image: %s", exc)
            return

        with self.lock:
            self.depth_image = np.array(depth, copy=True)
            self.depth_encoding = msg.encoding
            self.depth_stamp = msg.header.stamp
            self.depth_frames.append(
                (msg.header.stamp, self.depth_image, msg.encoding)
            )

    def color_callback(self, msg):
        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert D435 color image: %s", exc)
            return

        with self.lock:
            self.color_image = np.array(color, copy=True)
            self.color_header = msg.header
            self.color_frames.append(
                (msg.header.stamp, self.color_image, msg.header)
            )
            if self.first_color_wall_time is None:
                self.first_color_wall_time = time.monotonic()
                if self.d435_exposure_warmup_seconds > 0.0:
                    rospy.loginfo(
                        "D435 mapping image exposure warm-up: %.1f s",
                        self.d435_exposure_warmup_seconds,
                    )

    def matched_d435_frames(self, stamp):
        """Select nearest received frames for the chosen thermal event.

        Callback arrays are replaced, never edited, so retaining references
        here gives the entire fusion operation a consistent input snapshot.
        """
        with self.lock:
            depth_frames = list(self.depth_frames)
            color_frames = list(self.color_frames)

        def nearest(frames):
            if not frames:
                return None
            if stamp is None or stamp == rospy.Time(0):
                return frames[-1]
            return min(frames, key=lambda frame: abs((frame[0] - stamp).to_sec()))

        depth = nearest(depth_frames)
        color = nearest(color_frames)
        return (
            depth[1] if depth else None,
            depth[2] if depth else None,
            depth[0] if depth else None,
            color[1] if color else None,
            color[2] if color else None,
        )

    def d435_exposure_ready(self, now=None):
        """Return true after D435 auto exposure has had time to settle."""
        if self.d435_exposure_warmup_seconds <= 0.0:
            return True
        if self.first_color_wall_time is None:
            return False
        current_time = time.monotonic() if now is None else float(now)
        ready = (
            current_time - float(self.first_color_wall_time)
            >= self.d435_exposure_warmup_seconds
        )
        if ready and not self.d435_exposure_warmup_finished:
            self.d435_exposure_warmup_finished = True
            rospy.loginfo("D435 mapping image exposure warm-up finished")
        return ready

    def publish_valid(self, valid):
        self.fusion_valid_pub.publish(Bool(data=bool(valid)))

    def map_thermal_to_d435(self, u_t, v_t):
        thermal_pt = np.array([[[float(u_t), float(v_t)]]], dtype=np.float32)
        d435_pt = cv2.perspectiveTransform(thermal_pt, self.homography.astype(np.float32))
        u_d = int(round(float(d435_pt[0, 0, 0])))
        v_d = int(round(float(d435_pt[0, 0, 1])))
        return u_d, v_d

    def map_thermal_bbox_to_d435(self, bbox):
        if bbox is None:
            return None
        x, y, width, height = bbox
        corners = np.array(
            [[[x, y], [x + width, y], [x + width, y + height], [x, y + height]]],
            dtype=np.float32,
        )
        mapped = cv2.perspectiveTransform(corners, self.homography.astype(np.float32))[0]
        return np.rint(mapped).astype(np.int32)

    def depth_to_meters(self, value, encoding=None):
        if value is None:
            return None
        depth = float(value)
        if math.isnan(depth) or math.isinf(depth):
            return None
        encoding = self.depth_encoding if encoding is None else encoding
        if encoding in ("16UC1", "mono16"):
            depth *= self.depth_scale
        elif encoding == "32FC1":
            pass
        else:
            depth *= self.depth_scale
        return depth

    def lookup_depth(self, depth_image, u, v, encoding=None):
        h, w = depth_image.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        r = max(0, self.depth_search_radius)
        x0 = max(0, u - r)
        x1 = min(w, u + r + 1)
        y0 = max(0, v - r)
        y1 = min(h, v + r + 1)

        valid_depths = []
        for yy in range(y0, y1):
            for xx in range(x0, x1):
                depth = self.depth_to_meters(depth_image[yy, xx], encoding)
                if depth is None:
                    continue
                if self.min_valid_depth <= depth <= self.max_valid_depth:
                    valid_depths.append(depth)

        if not valid_depths:
            return None
        return float(np.median(np.array(valid_depths, dtype=np.float32)))

    def lookup_bbox_foreground_depth(self, depth_image, thermal_bbox, encoding=None):
        mapped_bbox = self.map_thermal_bbox_to_d435(thermal_bbox)
        if mapped_bbox is None:
            return None

        image_height, image_width = depth_image.shape[:2]
        x, y, width, height = cv2.boundingRect(mapped_bbox)
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(image_width, x + width)
        y1 = min(image_height, y + height)
        if x0 >= x1 or y0 >= y1:
            return None

        local_bbox = mapped_bbox - np.array([x0, y0], dtype=np.int32)
        mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        cv2.fillPoly(mask, [local_bbox.reshape((-1, 1, 2))], 1)
        raw_depths = depth_image[y0:y1, x0:x1][mask != 0].astype(np.float32)
        if raw_depths.size == 0:
            return None

        encoding = self.depth_encoding if encoding is None else encoding
        if encoding in ("16UC1", "mono16"):
            depths = raw_depths * self.depth_scale
        elif encoding == "32FC1":
            depths = raw_depths
        else:
            depths = raw_depths * self.depth_scale

        depths = depths[
            np.isfinite(depths)
            & (depths >= self.min_valid_depth)
            & (depths <= self.max_valid_depth)
        ]
        if depths.size == 0:
            return None

        bin_size = 0.2
        bin_indices = np.floor(
            (depths - self.min_valid_depth) / bin_size
        ).astype(np.int32)
        counts = np.bincount(bin_indices)
        min_cluster_size = max(5, int(math.ceil(depths.size * 0.05)))
        foreground_bins = np.flatnonzero(counts >= min_cluster_size)
        if foreground_bins.size == 0:
            return None

        foreground_depths = depths[bin_indices == int(foreground_bins[0])]
        return float(np.median(foreground_depths))

    def lookup_target_depth(
        self, depth_image, u, v, thermal_bbox=None, encoding=None
    ):
        depth = self.lookup_depth(depth_image, u, v, encoding)
        if depth is not None:
            return depth, "center"
        depth = self.lookup_bbox_foreground_depth(
            depth_image, thermal_bbox, encoding
        )
        return depth, "bbox" if depth is not None else None

    def pixel_to_camera_point(self, u, v, depth, camera_info, stamp):
        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])

        if fx == 0.0 or fy == 0.0:
            return None

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.d435_optical_frame
        point.point.x = (float(u) - cx) * depth / fx
        point.point.y = (float(v) - cy) * depth / fy
        point.point.z = depth
        return point

    def publish_d435_debug_image(
        self,
        color_image,
        color_header,
        detected,
        thermal_pixel_msg,
        candidate_detected,
        candidate_pixel_msg,
        thermal_bbox,
        camera_info,
        depth_image,
        depth_encoding,
        output_stamp=None,
        publish_realtime=False,
        publish_evidence=False,
    ):
        if color_image is None:
            return

        debug_image = color_image.copy()
        source = None
        status = "NO THERMAL TARGET"
        if detected and thermal_pixel_msg is not None and thermal_pixel_msg.point.z > 0.0:
            source = thermal_pixel_msg
            status = "CONFIRMED"
        elif (
            candidate_detected
            and candidate_pixel_msg is not None
            and candidate_pixel_msg.point.z > 0.0
        ):
            source = candidate_pixel_msg
            status = "CANDIDATE"

        if source is None:
            cv2.putText(
                debug_image,
                status,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )
        else:
            u_t = float(source.point.x)
            v_t = float(source.point.y)
            u_d, v_d = self.map_thermal_to_d435(u_t, v_t)
            depth = None
            depth_source = None
            if depth_image is not None and depth_encoding is not None:
                depth, depth_source = self.lookup_target_depth(
                    depth_image,
                    u_d,
                    v_d,
                    thermal_bbox,
                    depth_encoding,
                )

            height, width = debug_image.shape[:2]
            in_image = 0 <= u_d < width and 0 <= v_d < height
            color = (0, 255, 0) if depth is not None and in_image else (0, 0, 255)
            if in_image:
                mapped_bbox = self.map_thermal_bbox_to_d435(thermal_bbox)
                if mapped_bbox is not None:
                    cv2.polylines(
                        debug_image,
                        [mapped_bbox.reshape((-1, 1, 2))],
                        True,
                        color,
                        2,
                    )
                cv2.circle(
                    debug_image,
                    (u_d, v_d),
                    max(1, self.depth_search_radius),
                    color,
                    2,
                )
                cv2.drawMarker(
                    debug_image,
                    (u_d, v_d),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=24,
                    thickness=2,
                )

            depth_text = "invalid" if depth is None else "%.3f m" % depth
            if depth_source == "bbox":
                depth_text += " (bbox)"
            cv2.putText(
                debug_image,
                "%s thermal=(%.1f,%.1f) d435=(%d,%d) depth=%s"
                % (status, u_t, v_t, u_d, v_d, depth_text),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
            camera_point = None
            if depth is not None and camera_info is not None:
                camera_point = self.pixel_to_camera_point(
                    u_d, v_d, depth, camera_info, source.header.stamp
                )
            if camera_point is not None:
                cv2.putText(
                    debug_image,
                    "D435 XYZ=(%.3f, %.3f, %.3f) m"
                    % (
                        camera_point.point.x,
                        camera_point.point.y,
                        camera_point.point.z,
                    ),
                    (20, 72),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        if color_header is not None:
            debug_msg.header = deepcopy(color_header)
        if output_stamp is not None:
            debug_msg.header.stamp = output_stamp
        if publish_realtime:
            self.d435_debug_image_pub.publish(debug_msg)
        if publish_evidence:
            self.d435_evidence_image_pub.publish(debug_msg)

    def build_camera_point(
        self,
        thermal_pixel_msg,
        camera_info,
        depth_image,
        depth_encoding,
        thermal_bbox=None,
    ):
        """Fuse a thermal pixel with the supplied matched depth, without TF."""
        if (
            thermal_pixel_msg is None
            or thermal_pixel_msg.point.z <= 0.0
            or camera_info is None
            or depth_image is None
            or depth_encoding is None
        ):
            return None
        u_d, v_d = self.map_thermal_to_d435(
            thermal_pixel_msg.point.x, thermal_pixel_msg.point.y
        )
        depth, _depth_source = self.lookup_target_depth(
            depth_image, u_d, v_d, thermal_bbox, depth_encoding
        )
        if depth is None:
            return None
        return self.pixel_to_camera_point(
            u_d, v_d, depth, camera_info, thermal_pixel_msg.header.stamp
        )

    @staticmethod
    def status_candidates(status):
        if not isinstance(status, dict):
            return []
        candidates = status.get("candidates")
        if isinstance(candidates, list):
            return [item for item in candidates if isinstance(item, dict)]
        if status.get("detected") and status.get("cx") is not None:
            return [status]
        return []

    def publish_report_candidates(
        self, status, camera_info, depth_image, depth_encoding,
        color_image=None, color_header=None, depth_stamp=None
    ):
        if status is None or camera_info is None or depth_image is None:
            return False
        if depth_encoding is None:
            return False
        try:
            base_stamp = float(status.get("stamp"))
        except (TypeError, ValueError):
            base_stamp = rospy.Time.now().to_sec()

        color_stamp = color_header.stamp.to_sec() if color_header is not None else None
        depth_time = depth_stamp.to_sec() if depth_stamp is not None else None
        source_timing = {
            "thermal_stamp": base_stamp,
            "color_stamp": color_stamp,
            "depth_stamp": depth_time,
            "color_dt_s": None if color_stamp is None else color_stamp - base_stamp,
            "depth_dt_s": None if depth_time is None else depth_time - base_stamp,
        }
        fusion_stamp = rospy.Time.now().to_sec()
        source_timing["fusion_stamp"] = fusion_stamp
        source_timing["fusion_age_s"] = fusion_stamp - base_stamp
        source_timing["color_depth_dt_s"] = (
            None if color_stamp is None or depth_time is None else color_stamp - depth_time
        )
        if color_stamp is not None and depth_time is not None:
            rospy.loginfo_throttle(
                1.0, "Thermal frame match: color_dt=%.1f ms depth_dt=%.1f ms fusion_age=%.1f ms",
                source_timing["color_dt_s"] * 1000.0,
                source_timing["depth_dt_s"] * 1000.0,
                source_timing["fusion_age_s"] * 1000.0,
            )

        for index, candidate in enumerate(self.status_candidates(status)):
            try:
                cx = float(candidate["cx"])
                cy = float(candidate["cy"])
            except (KeyError, TypeError, ValueError):
                continue
            stamp = base_stamp + index * 1.0e-6
            thermal_point = PointStamped()
            thermal_point.header.stamp = rospy.Time.from_sec(stamp)
            thermal_point.header.frame_id = "thermal_camera"
            thermal_point.point.x = cx
            thermal_point.point.y = cy
            thermal_point.point.z = 1.0
            camera_point = self.build_camera_point(
                thermal_point,
                camera_info,
                depth_image,
                depth_encoding,
                candidate.get("bbox"),
            )
            if camera_point is None:
                continue
            camera_point.header.stamp = thermal_point.header.stamp

            # Give each candidate its own same-frame D435 evidence image so
            # filtering one candidate cannot leave its box in another one's
            # locally saved or remotely uploaded image.
            if color_image is not None:
                self.publish_d435_debug_image(
                    color_image,
                    color_header,
                    False,
                    None,
                    True,
                    thermal_point,
                    candidate.get("bbox"),
                    camera_info,
                    depth_image,
                    depth_encoding,
                    output_stamp=thermal_point.header.stamp,
                    publish_evidence=True,
                )

            stable = bool(candidate.get("stable", status.get("stable", False)))
            report_status = {
                "detected": True,
                "candidate": True,
                "stable": stable,
                "confirmable": bool(candidate.get("confirmable", stable)),
                "stable_count": int(candidate.get("stable_count", 0)),
                "stable_window": int(status.get("stable_window", 1)),
                "stamp": stamp,
                "source_timing": source_timing,
                "rank": int(candidate.get("rank", index + 1)),
                "cx": int(round(cx)),
                "cy": int(round(cy)),
                "bbox": candidate.get("bbox"),
                "area": float(candidate.get("area", 0.0)),
                "touches_roi_border": bool(
                    candidate.get("touches_roi_border", False)
                ),
                "camera_point": {
                    "frame_id": camera_point.header.frame_id,
                    "x": camera_point.point.x,
                    "y": camera_point.point.y,
                    "z": camera_point.point.z,
                },
            }
            self.report_candidate_pub.publish(
                String(data=json.dumps(report_status, ensure_ascii=False))
            )
        return True

    def is_ground_candidate(self, camera_point):
        if not self.thermal_ground_filter_enabled or not self.use_tf:
            return False
        try:
            world_point = self.tf_buffer.transform(
                camera_point, self.target_frame, rospy.Duration(0.05)
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Cannot transform thermal candidate for ground filter: %s", exc
            )
            return False
        if world_point.point.z >= self.thermal_min_height:
            return False
        rospy.logwarn_throttle(
            1.0,
            "Skip ground thermal candidate in D435 mapping: z=%.3fm min_height=%.3fm",
            world_point.point.z,
            self.thermal_min_height,
        )
        return True

    def select_display_candidate(
        self, status, camera_info, depth_image, depth_encoding
    ):
        """Return the highest-ranked non-ground candidate for the mapping view."""
        try:
            source_stamp = rospy.Time.from_sec(float(status.get("stamp")))
        except (TypeError, ValueError):
            source_stamp = rospy.Time.now()
        for candidate in self.status_candidates(status):
            try:
                cx = float(candidate["cx"])
                cy = float(candidate["cy"])
            except (KeyError, TypeError, ValueError):
                continue
            thermal_point = PointStamped()
            thermal_point.header.stamp = source_stamp
            thermal_point.header.frame_id = "thermal_camera"
            thermal_point.point.x = cx
            thermal_point.point.y = cy
            thermal_point.point.z = 1.0
            camera_point = self.build_camera_point(
                thermal_point,
                camera_info,
                depth_image,
                depth_encoding,
                candidate.get("bbox"),
            )
            if camera_point is None or self.is_ground_candidate(camera_point):
                continue
            return thermal_point, candidate.get("bbox")
        return None

    def timer_callback(self, _event):
        with self.lock:
            detected = self.target_detected
            thermal_pixel_msg = self.thermal_pixel_msg
            candidate_detected = self.candidate_target_detected
            candidate_pixel_msg = self.candidate_thermal_pixel_msg
            candidate_thermal_bbox = self.candidate_thermal_bbox
            candidate_status, candidate_status_version = self.fusion_status_snapshot(
                rospy.Time.now()
            )
            camera_info = self.camera_info

        if self.fusion_delay_s > 0.0:
            if candidate_status is None:
                return  # Startup: thermal events are still waiting for their frames.
            # Do not combine a delayed event with the latest Bool/Point messages.
            detected = False
            thermal_pixel_msg = None
            candidates = self.status_candidates(candidate_status)
            candidate_detected = bool(candidates)
            candidate_pixel_msg = None
            candidate_thermal_bbox = None
            if candidates:
                candidate = candidates[0]
                candidate_pixel_msg = PointStamped()
                candidate_pixel_msg.header.stamp = rospy.Time.from_sec(
                    float(candidate_status["stamp"])
                )
                candidate_pixel_msg.header.frame_id = "thermal_camera"
                candidate_pixel_msg.point.x = float(candidate["cx"])
                candidate_pixel_msg.point.y = float(candidate["cy"])
                candidate_pixel_msg.point.z = 1.0
                candidate_thermal_bbox = candidate.get("bbox")

        try:
            status_stamp = rospy.Time.from_sec(float(candidate_status["stamp"]))
        except (TypeError, ValueError, KeyError):
            status_stamp = None
        report_depth, report_encoding, report_depth_stamp, report_color, report_header = (
            self.matched_d435_frames(status_stamp)
        )
        has_candidates = bool(self.status_candidates(candidate_status))
        if has_candidates:
            depth_image, depth_encoding = report_depth, report_encoding
            color_image, color_header = report_color, report_header
        else:
            display_stamp = (
                thermal_pixel_msg.header.stamp
                if detected and thermal_pixel_msg is not None else None
            )
            depth_image, depth_encoding, _, color_image, color_header = (
                self.matched_d435_frames(display_stamp)
            )

        display_detected = detected
        display_thermal_pixel = thermal_pixel_msg
        display_bbox = candidate_thermal_bbox
        if has_candidates:
            selected = self.select_display_candidate(
                candidate_status, camera_info, depth_image, depth_encoding
            )
            display_detected = selected is not None
            display_thermal_pixel = selected[0] if selected is not None else None
            display_bbox = selected[1] if selected is not None else None

        # The RealSense color sensor starts with a few dark auto-exposure
        # frames.  Keep fusion/depth processing active, but do not put those
        # frames into the RViz mapping panel or saved/remote evidence.
        if self.d435_exposure_ready():
            self.publish_d435_debug_image(
                color_image,
                color_header,
                display_detected,
                display_thermal_pixel,
                False,
                None,
                display_bbox,
                camera_info,
                depth_image,
                depth_encoding,
                publish_realtime=True,
            )

        if (
            self.report_candidate_pub is not None
            and candidate_status_version > self.published_candidate_status_version
        ):
            if self.publish_report_candidates(
                candidate_status,
                camera_info,
                report_depth,
                report_encoding,
                report_color,
                report_header,
                depth_stamp=report_depth_stamp,
            ):
                self.published_candidate_status_version = candidate_status_version

        # Candidate fusion is independent of the package's stable 2/3-frame
        # stream. It only requires a valid raw hotspot pixel and valid depth;
        # target_reporting performs the later world-frame tracking/confirmation.
        if candidate_detected:
            candidate_stamp = candidate_pixel_msg.header.stamp if candidate_pixel_msg else None
            candidate_depth, candidate_encoding, _, _, _ = self.matched_d435_frames(
                candidate_stamp
            )
            candidate_camera_point = self.build_camera_point(
                candidate_pixel_msg,
                camera_info,
                candidate_depth,
                candidate_encoding,
                candidate_thermal_bbox,
            )
            if candidate_camera_point is not None:
                self.candidate_camera_point_pub.publish(candidate_camera_point)

        if (
            not display_detected
            or display_thermal_pixel is None
            or display_thermal_pixel.point.z <= 0.0
        ):
            self.publish_valid(False)
            return

        if camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for D435 camera_info")
            self.publish_valid(False)
            return

        if depth_image is None or depth_encoding is None:
            rospy.logwarn_throttle(2.0, "Waiting for aligned D435 depth image")
            self.publish_valid(False)
            return

        u_t = display_thermal_pixel.point.x
        v_t = display_thermal_pixel.point.y
        u_d, v_d = self.map_thermal_to_d435(u_t, v_t)

        depth, depth_source = self.lookup_target_depth(
            depth_image,
            u_d,
            v_d,
            display_bbox,
            depth_encoding,
        )
        d435_pixel = PointStamped()
        d435_pixel.header.stamp = display_thermal_pixel.header.stamp
        d435_pixel.header.frame_id = "d435_color_image"
        d435_pixel.point.x = float(u_d)
        d435_pixel.point.y = float(v_d)
        d435_pixel.point.z = 0.0 if depth is None else float(depth)
        self.d435_pixel_pub.publish(d435_pixel)

        if depth is None:
            rospy.logwarn_throttle(
                2.0,
                "Invalid depth near D435 pixel (%d, %d); search_radius=%d",
                u_d,
                v_d,
                self.depth_search_radius,
            )
            self.publish_valid(False)
            return

        if depth_source == "bbox":
            rospy.loginfo_throttle(
                2.0,
                "Using thermal bbox foreground depth at D435 pixel (%d, %d): %.3f m",
                u_d,
                v_d,
                depth,
            )

        camera_point = self.pixel_to_camera_point(
            u_d, v_d, depth, camera_info, display_thermal_pixel.header.stamp
        )
        if camera_point is None:
            rospy.logwarn_throttle(2.0, "Invalid D435 camera intrinsics")
            self.publish_valid(False)
            return

        self.camera_point_pub.publish(camera_point)

        if not self.use_tf:
            channel_point = PointStamped()
            channel_point.header = camera_point.header
            channel_point.point = camera_point.point
            self.channel_position_pub.publish(channel_point)
            self.publish_valid(True)
            return

        try:
            channel_point = self.tf_buffer.transform(
                camera_point, self.target_frame, rospy.Duration(0.05)
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "TF transform failed from %s to %s: %s",
                camera_point.header.frame_id,
                self.target_frame,
                exc,
            )
            self.publish_valid(False)
            return

        self.channel_position_pub.publish(channel_point)
        self.publish_valid(True)

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "thermal=(%.1f, %.1f) d435=(%d, %d) depth=%.3f %s=(%.3f, %.3f, %.3f)",
                u_t,
                v_t,
                u_d,
                v_d,
                depth,
                channel_point.header.frame_id,
                channel_point.point.x,
                channel_point.point.y,
                channel_point.point.z,
            )


def main():
    rospy.init_node("thermal_d435_fusion_node", anonymous=False)
    ThermalD435FusionNode()
    rospy.loginfo("thermal_d435_fusion_node started")
    rospy.spin()


if __name__ == "__main__":
    main()
