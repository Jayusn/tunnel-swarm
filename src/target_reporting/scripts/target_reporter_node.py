#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from collections import defaultdict, deque

import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from target_reporting.adapters import parse_color_status, parse_qr_status, parse_thermal_status
from target_reporting.candidates import CandidateTracker
from target_reporting.evidence import (
    TimestampedImageCache,
    build_evidence_jpeg,
    draw_detection_overlay,
    image_metadata,
)
from target_reporting.model import Position, TargetEvent
from target_reporting.store import MissionStore
from target_reporting.transport import ImageAckClient, JsonAckClient, LatestJsonClient


class TargetReporterNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.candidate_lock = threading.Lock()
        self.drone_id = rospy.get_param("~drone_id", "uav1")
        self.localization_frame = rospy.get_param("~localization_frame", "camera_init")
        self.report_frame = rospy.get_param("~report_frame", "channel")
        # p_channel = R_channel_world * p_world + t_channel_world (metres).
        self.world_to_channel_rotation = rospy.get_param(
            "~world_to_channel/rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.world_to_channel_translation = rospy.get_param(
            "~world_to_channel/translation", [0.0, 0.0, 0.0]
        )
        self.use_detector_candidates = bool(
            rospy.get_param("~use_detector_candidates", True)
        )
        self.save_unlocalized_evidence = bool(
            rospy.get_param("~save_unlocalized_evidence", False)
        )
        self.wait_for_entry_goal = bool(rospy.get_param("~wait_for_entry_goal", False))
        self.recording_start_stamp = None
        self.pending_unlocalized = defaultdict(lambda: deque(maxlen=40))
        self.unlocalized_save_interval = max(0.0, float(rospy.get_param("~unlocalized_save_interval_s", 5.0)))
        self.unlocalized_center_shift = max(0.0, float(rospy.get_param("~unlocalized_center_shift_px", 80.0)))
        self.thermal_evidence_image_topic = rospy.get_param(
            "~thermal_evidence_image_topic", "/UAV0/thermal/image_raw"
        )
        self.thermal_fused_topic = str(rospy.get_param("~thermal_fused_candidate_topic", "")).strip()
        self.exit_plane_filter_enabled = bool(
            rospy.get_param("~exit_plane_filter_enabled", False)
        )
        self.exit_plane = None
        # A confirmed event may only use the detector debug image carrying the
        # same source timestamp.  The old broad 0.5 s lookup could select a
        # preceding raw-candidate frame while the stable frame was in flight.
        self.image_tolerance = float(
            rospy.get_param(
                "~image_match_tolerance_s",
                rospy.get_param("~image_tolerance_s", 0.03),
            )
        )
        self.image_wait_timeout = max(
            0.0, float(rospy.get_param("~image_wait_timeout_s", 0.50))
        )
        self.d435_image_tolerance = max(
            0.0,
            float(rospy.get_param("~d435_image_match_tolerance_s", 0.10)),
        )
        self.require_thermal_d435_evidence = bool(
            rospy.get_param("~require_thermal_d435_evidence", True)
        )
        self.leader_thermal_filter_enabled = bool(
            rospy.get_param("~leader_thermal_filter_enabled", False)
        )
        self.thermal_ground_filter_enabled = bool(
            rospy.get_param("~thermal_ground_filter_enabled", False)
        )
        self.thermal_min_height = float(
            rospy.get_param("~thermal_min_height_m", -0.08)
        )
        self.leader_thermal_exclusion_radius = max(
            0.0,
            float(rospy.get_param("~leader_thermal_exclusion_radius_m", 0.35)),
        )
        self.leader_odom_max_age = max(
            0.0, float(rospy.get_param("~leader_odom_max_age_s", 0.50))
        )
        self.leader_odom_match_tolerance = max(
            0.0, float(rospy.get_param("~leader_odom_match_tolerance_s", 0.06))
        )
        self.leader_thermal_projection_filter_enabled = bool(
            rospy.get_param("~leader_thermal_projection_filter_enabled", False)
        )
        self.leader_thermal_projection_radius = max(
            0.0, float(rospy.get_param("~leader_thermal_projection_radius_m", 0.35))
        )
        self.leader_odom_frame = str(
            rospy.get_param("~leader_odom_frame", "")
        ).strip()
        self.thermal_d435_ready_tolerance = max(
            self.d435_image_tolerance,
            float(rospy.get_param("~thermal_d435_ready_tolerance_s", 0.50)),
        )
        self.source_match_tolerance = max(
            0.0,
            float(
                rospy.get_param(
                    "~source_match_tolerance_s", self.image_tolerance
                )
            ),
        )
        configured_mission_id = str(rospy.get_param("~mission_id", "")).strip()
        if not configured_mission_id or configured_mission_id.lower() == "auto":
            if self.drone_id == "uav1":
                configured_mission_id = time.strftime("%Y%m%d/flight_%H%M%S")
            else:
                configured_mission_id = "onboard_test_{}".format(
                    time.strftime("%Y%m%d")
                )
        self.mission_id = configured_mission_id
        self.store = MissionStore(
            rospy.get_param("~record_root", "~/target_reports"), self.drone_id,
            mission_id=self.mission_id,
        )
        for target_type in self.store.IMAGE_TARGET_TYPES:
            os.makedirs(os.path.join(self.store.image_dir, target_type), exist_ok=True)
        self.last_unlocalized = {}
        saved_unlocalized = self.store.unlocalized_events()
        for event in saved_unlocalized:
            self.last_unlocalized[event["target_type"]] = event
        self.unlocalized_seq = max(
            [len(saved_unlocalized)] + [int(e.get("evidence_seq", 0)) for e in saved_unlocalized]
        )
        self.tracker = CandidateTracker(
            rospy.get_param("~dedup_distance_m", 0.30),
            rospy.get_param("~confirm_hits", 1),
            confirm_hits_by_type={
                "thermal_source": rospy.get_param("~thermal_confirm_hits", 20)
            },
            confirmation_max_gap_s_by_type={
                "thermal_source": rospy.get_param(
                    "~thermal_confirmation_max_gap_s", 0.25
                )
            },
            confirmation_duration_s_by_type={
                "thermal_source": rospy.get_param(
                    "~thermal_confirmation_duration_s", 1.0
                )
            },
            color_image_distance_px=rospy.get_param(
                "~color_image_dedup_distance_px", 80.0
            ),
            color_image_max_gap_s=rospy.get_param(
                "~color_image_dedup_max_gap_s", 1.0
            ),
        )
        self.images = TimestampedImageCache(rospy.get_param("~image_cache_items", 40))
        self.enable_remote_upload = bool(
            rospy.get_param("~enable_remote_upload", True)
        )
        if self.enable_remote_upload:
            host = rospy.get_param("~remote_host", "192.168.10.100")
            remote_port = rospy.get_param("~remote_port", 5000)
            self.json_client = JsonAckClient(
                host, remote_port, ack_callback=self.store.mark_event_acked
            )
            self.image_client = ImageAckClient(
                host, rospy.get_param("~image_port", 5001),
                ack_callback=self.store.mark_image_acked,
            )
            self.realtime_client = LatestJsonClient(host, remote_port)
            self.json_client.start()
            self.image_client.start()
            self.realtime_client.start()
        rospy.loginfo(
            "target_reporter: mission_id=%s enable_remote_upload=%s confirmed-only=true",
            self.mission_id, self.enable_remote_upload,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.observation_pub = rospy.Publisher("/UAV0/target_reporting/observation", String, queue_size=10)
        if self.wait_for_entry_goal:
            self.entry_goal_sub = rospy.Subscriber(
                rospy.get_param("~entry_goal_topic", "/UAV1/planning/goal/entry"),
                PoseStamped, self._entry_goal_cb, queue_size=1,
            )
        self.retry_timer = None
        self.latest_point = {}
        self.point_history = defaultdict(lambda: deque(maxlen=30))
        self.latest_status = {}
        self.point_version = {}
        self.status_version = {}
        self.last_processed_pair = {}
        # 200 Hz 前机里程计保留约 2 秒，用检测时间匹配，避免处理延迟引入位置差。
        self.leader_odom_history = deque(maxlen=400)
        self.latest_leader_odom_received = rospy.Time(0)
        self.seq = 0
        existing = self.store.all_events()
        if existing:
            self.seq = max(int(event["seq"]) for event in existing)
        if self.enable_remote_upload:
            for event in self.store.pending_events():
                self.json_client.enqueue(event)
            for image_item in self.store.pending_images():
                self.image_client.enqueue(image_item)

        if self.use_detector_candidates:
            self._subscribe(
                "color",
                rospy.get_param(
                    "~color_candidate_point_topic",
                    "/UAV0/color_tag_detector/candidate_point_camera",
                ),
                PointStamped,
                rospy.get_param(
                    "~color_candidate_status_topic",
                    "/UAV0/color_tag_detector/candidate_text",
                ),
                String,
                "/UAV0/color_tag_detector/debug_image",
            )
            self._subscribe(
                "qr",
                rospy.get_param(
                    "~qr_candidate_point_topic",
                    "/UAV0/vision/qr_candidate_pose_camera",
                ),
                PoseStamped,
                rospy.get_param(
                    "~qr_candidate_status_topic",
                    "/UAV0/vision/qr_candidate_detected",
                ),
                String,
                "/UAV0/vision/qr_debug_image",
            )
            thermal_fused_topic = self.thermal_fused_topic
            if thermal_fused_topic:
                rospy.Subscriber(
                    thermal_fused_topic,
                    String,
                    self._thermal_fused_candidate_cb,
                    queue_size=10,
                )
                rospy.Subscriber(
                    self.thermal_evidence_image_topic,
                    Image,
                    lambda msg: self._image_cb("thermal", msg),
                    queue_size=2,
                )
            else:
                self._subscribe(
                    "thermal",
                    rospy.get_param(
                        "~thermal_candidate_point_topic",
                        "/UAV0/thermal/target_candidate_camera_point",
                    ),
                    PointStamped,
                    rospy.get_param(
                        "~thermal_candidate_status_topic",
                        "/UAV0/thermal/target_candidate_status",
                    ),
                    String,
                    self.thermal_evidence_image_topic,
                )
        else:
            # Compatibility path for old detector nodes without candidate topics.
            self._subscribe(
                "color", "/UAV0/color_tag_detector/target_point_camera", PointStamped,
                "/UAV0/color_tag_detector/result_text", String, "/UAV0/color_tag_detector/debug_image"
            )
            self._subscribe(
                "qr", "/UAV0/vision/qr_pose_camera", PoseStamped,
                "/UAV0/vision/qr_detected", String, "/UAV0/vision/qr_debug_image"
            )
            self._subscribe(
                "thermal", "/UAV0/thermal/target_camera_point", PointStamped,
                "/UAV0/thermal/fusion_valid", Bool, self.thermal_evidence_image_topic
            )
        rospy.Subscriber(
            rospy.get_param(
                "~thermal_d435_debug_image_topic",
                "/UAV0/thermal/d435_debug_image",
            ),
            Image,
            lambda msg: self._image_cb("thermal_d435", msg),
            queue_size=2,
        )
        if self.leader_thermal_filter_enabled:
            rospy.Subscriber(
                rospy.get_param(
                    "~leader_odom_topic", "/UAV0/fast_lio/Odom_high_freq"
                ),
                Odometry,
                self._leader_odom_cb,
                queue_size=1,
            )
        if self.exit_plane_filter_enabled:
            rospy.Subscriber(
                rospy.get_param("~final_exit_pose_topic", "/UAV0/mission/final_exit"),
                PoseStamped,
                self._final_exit_pose_cb,
                queue_size=1,
            )
        # TF 可能在节点启动后的短时间内尚未建立。候选不能因为一次 TF
        # 查询失败就丢失，因此定时重试尚未处理的点/状态配对。
        self.retry_timer = rospy.Timer(rospy.Duration(0.10), self._retry_pending)
        rospy.on_shutdown(self.shutdown)

    def _entry_goal_cb(self, _msg):
        with self.lock:
            if self.recording_start_stamp is not None:
                return
            self.recording_start_stamp = rospy.Time.now().to_sec()
        rospy.loginfo("Target recording enabled by entry goal; earlier events will not be saved")

    def _recording_allowed(self, event_stamp):
        if not self.wait_for_entry_goal:
            return True
        return (self.recording_start_stamp is not None
                and event_stamp >= self.recording_start_stamp)

    def _subscribe(self, source, point_topic, point_type, status_topic, status_type, image_topic):
        rospy.Subscriber(point_topic, point_type, lambda msg: self._point_cb(source, msg), queue_size=5)
        rospy.Subscriber(status_topic, status_type, lambda msg: self._status_cb(source, msg), queue_size=5)
        rospy.Subscriber(image_topic, Image, lambda msg: self._image_cb(source, msg), queue_size=2)

    def _point_cb(self, source, msg):
        point = msg
        if isinstance(msg, PoseStamped):
            point = PointStamped(header=msg.header)
            point.point = msg.pose.position
        with self.lock:
            self.latest_point[source] = point
            self.point_history[source].append(point)
            self.point_version[source] = self.point_version.get(source, 0) + 1
        self._try_process(source)

    def _status_cb(self, source, msg):
        try:
            if isinstance(msg.data, str):
                data = json.loads(msg.data)
                if data.get("localization_valid") is False:
                    self._queue_unlocalized(source, data)
                    if source != "qr" or not all(data.get(axis) is not None for axis in ("x", "y", "z")):
                        return
            if source == "color":
                status = parse_color_status(msg.data)
            elif source == "qr":
                status = parse_qr_status(msg.data)
            else:
                status = parse_thermal_status(msg.data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid %s status: %s", source, exc)
            return
        with self.lock:
            self.latest_status[source] = status
            self.status_version[source] = self.status_version.get(source, 0) + 1
        self._try_process(source)

    def _thermal_fused_candidate_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("localization_valid") is False:
                self._queue_unlocalized("thermal", data)
                return
            camera_point = data["camera_point"]
            frame_id = str(camera_point["frame_id"]).strip()
            stamp = float(data["stamp"])
            coordinates = [
                float(camera_point[axis]) for axis in ("x", "y", "z")
            ]
            if not frame_id or not math.isfinite(stamp) or not all(
                math.isfinite(value) for value in coordinates
            ):
                raise ValueError("invalid fused thermal point")
            status = parse_thermal_status(msg.data)
            if status is None:
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid fused thermal candidate: %s", exc)
            return

        point = PointStamped()
        point.header.stamp = rospy.Time.from_sec(stamp)
        point.header.frame_id = frame_id
        point.point.x, point.point.y, point.point.z = coordinates
        with self.lock:
            self.latest_point["thermal"] = point
            self.point_history["thermal"].append(point)
            self.point_version["thermal"] = self.point_version.get("thermal", 0) + 1
            self.latest_status["thermal"] = status
            self.status_version["thermal"] = self.status_version.get("thermal", 0) + 1
        self._try_process("thermal")

    def _final_exit_pose_cb(self, msg):
        # final_exit publishes the portal centre and its outward yaw, latched.
        # Keep it in the message frame; target points are transformed before
        # comparison, so the UAV0/UAV1 initial-position offset is not lost.
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not msg.header.frame_id or not all(map(math.isfinite, values)) or norm_sq <= 0:
            rospy.logwarn_throttle(2.0, "Invalid final exit pose; exit plane not updated")
            return
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            norm_sq - 2.0 * (q.y * q.y + q.z * q.z),
        )
        plane = (msg.header.frame_id, p.x, p.y, math.cos(yaw), math.sin(yaw))
        with self.lock:
            self.exit_plane = plane
        rospy.loginfo(
            "Exit plane filter ready: frame=%s center=(%.3f, %.3f) outward_yaw=%.1f deg",
            plane[0], p.x, p.y, math.degrees(yaw),
        )

    def _is_beyond_exit_plane(self, point):
        if not self.exit_plane_filter_enabled:
            return False
        with self.lock:
            plane = self.exit_plane
        if plane is None:
            return False
        frame, cx, cy, nx, ny = plane
        try:
            target = point if point.header.frame_id == frame else self.tf_buffer.transform(
                point, frame, rospy.Duration(0.10)
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Cannot compare target with exit plane: %s", exc)
            return False
        distance = (target.point.x - cx) * nx + (target.point.y - cy) * ny
        if distance <= 0.0:
            return False
        rospy.loginfo_throttle(
            1.0, "Filtered target outside final exit plane: distance=%.3f m frame=%s",
            distance, frame,
        )
        return True

    def _image_cb(self, source, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.images.add(source, msg.header.stamp.to_sec(), image)
            self._try_save_unlocalized(source)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Cannot cache %s image: %s", source, exc)

    def _queue_unlocalized(self, source, data):
        if (not self.save_unlocalized_evidence or not data.get("detected")
                or not data.get("evidence_credible") or data.get("held")):
            return
        stamp = data.get("stamp")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if not self._recording_allowed(stamp):
            return
        # A camera ray carries direction, not a made-up target distance. Only
        # the existing leader projection test can be evaluated without depth.
        projection = {"status": "incomplete"}
        if not (self.leader_thermal_filter_enabled and self.leader_thermal_projection_filter_enabled):
            projection["status"] = "disabled"
        if source == "thermal" and data.get("camera_ray"):
            ray = data["camera_ray"]
            point = PointStamped()
            point.header.frame_id = ray["frame_id"]
            point.header.stamp = rospy.Time.from_sec(stamp)
            point.point.x, point.point.y, point.point.z = (ray[k] for k in ("x", "y", "z"))
            if self._is_leader_thermal_source(None, stamp, point, projection):
                return
        data = dict(data)
        data["leader_projection_status"] = projection["status"] if source == "thermal" else "not_applicable"
        with self.lock:
            self.pending_unlocalized[source].append(data)
        self._try_save_unlocalized(source)

    @staticmethod
    def _evidence_result(data):
        result = {
            "content": data.get("data", ""), "color": data.get("color", ""),
            "points": data.get("points"), "bbox": data.get("bbox"),
            "center_u": data.get("center_u", data.get("u", data.get("cx"))),
            "center_v": data.get("center_v", data.get("v", data.get("cy"))),
        }
        if result["center_u"] is None or result["center_v"] is None:
            bbox = result["bbox"]
            if bbox is not None:
                result["center_u"] = bbox[0] + bbox[2] / 2.0
                result["center_v"] = bbox[1] + bbox[3] / 2.0
        return result

    def _unlocalized_due(self, target_type, stamp, result):
        previous = self.last_unlocalized.get(target_type)
        if previous is None:
            return True
        if stamp <= previous["timestamp"]:
            return False
        if stamp - previous["timestamp"] >= self.unlocalized_save_interval:
            return True
        old = previous["result"]
        coords = [result.get("center_u"), result.get("center_v"), old.get("center_u"), old.get("center_v")]
        if any(v is None for v in coords):
            return False
        return math.hypot(coords[0] - coords[2], coords[1] - coords[3]) >= self.unlocalized_center_shift

    def _try_save_unlocalized(self, source):
        if not self.save_unlocalized_evidence or source not in ("qr", "color", "thermal"):
            return
        target_type = {"qr": "qr_code", "color": "color_tag", "thermal": "thermal_source"}[source]
        with self.candidate_lock:
            with self.lock:
                pending = list(self.pending_unlocalized.get(source, ()))
            image = None
            for data in pending:
                stamp = float(data["stamp"])
                if not self._recording_allowed(stamp):
                    continue
                result = self._evidence_result(data)
                if not self._unlocalized_due(target_type, stamp, result):
                    continue
                image_stamp = data.get("source_timing", {}).get("thermal_stamp", stamp) if source == "thermal" else stamp
                image = self.images.nearest(source, image_stamp, 1e-7 if source == "thermal" else self.image_tolerance)
                if image is not None:
                    break
            if image is None:
                return
            self.unlocalized_seq += 1
            event = {
                "version": 1, "timestamp": stamp, "drone_id": self.drone_id,
                "target_type": target_type,
                "target_id": "{}-unlocalized-{}-{:06d}".format(self.drone_id, target_type, self.unlocalized_seq),
                "evidence_seq": self.unlocalized_seq,
                "message_type": "unlocalized_evidence", "position": None,
                "localization_status": "unlocalized", "localization_label": "未定位",
                "result": result,
                "spatial_filters": "not_evaluable_without_valid_position",
                "leader_projection_status": data["leader_projection_status"],
            }
            name = "{}_unlocalized_{:06d}_{}.jpg".format(self.drone_id, self.unlocalized_seq, target_type)
            jpeg = build_evidence_jpeg(draw_detection_overlay(image, event), event)
            self.store.save_image(name, jpeg, target_type=target_type)
            event["image"] = image_metadata(name, jpeg)
            self.store.append_unlocalized(event)
            self.last_unlocalized[target_type] = event
            with self.lock:
                self.pending_unlocalized[source] = deque(
                    (item for item in self.pending_unlocalized[source] if item["stamp"] > stamp), maxlen=40
                )
            rospy.loginfo("Saved local unlocalized evidence: %s", name)

    def _leader_odom_cb(self, msg):
        with self.lock:
            self.leader_odom_history.append(msg)
            self.latest_leader_odom_received = rospy.Time.now()

    def _is_leader_thermal_source(self, position, event_stamp, camera_point=None, projection=None):
        if not self.leader_thermal_filter_enabled:
            return False
        with self.lock:
            history = list(self.leader_odom_history)
            received = self.latest_leader_odom_received
        if not history or received == rospy.Time(0):
            return False
        if (rospy.Time.now() - received).to_sec() > self.leader_odom_max_age:
            return False
        leader_odom = min(
            history,
            key=lambda msg: abs(msg.header.stamp.to_sec() - event_stamp),
        )
        # 跨机里程计传输容差独立于检测点/状态/图像配对容差。
        match_delta = leader_odom.header.stamp.to_sec() - event_stamp
        if (
            leader_odom.header.stamp == rospy.Time(0)
            or abs(match_delta) > self.leader_odom_match_tolerance
        ):
            rospy.logwarn_throttle(
                2.0, "Skip leader thermal filter: odom_dt=%.1fms tolerance=%.1fms",
                match_delta * 1000.0, self.leader_odom_match_tolerance * 1000.0,
            )
            return False
        leader_frame = self.leader_odom_frame or leader_odom.header.frame_id
        if not leader_frame:
            return False

        leader_point = PointStamped()
        leader_point.header.frame_id = leader_frame
        leader_point.header.stamp = leader_odom.header.stamp
        leader_point.point = leader_odom.pose.pose.position
        try:
            leader = self.tf_buffer.transform(
                leader_point, self.localization_frame, rospy.Duration(0.10)
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Cannot transform leader odometry %s -> %s for thermal filter: %s",
                leader_point.header.frame_id,
                self.localization_frame,
                exc,
            )
            return False

        distance = None if position is None else math.hypot(
            position["x"] - leader.point.x,
            position["y"] - leader.point.y,
        )
        if distance is not None and distance <= self.leader_thermal_exclusion_radius:
            rospy.logwarn_throttle(
                1.0,
                "Filtered UAV1 thermal candidate near UAV0: distance=%.3fm radius=%.3fm",
                distance,
                self.leader_thermal_exclusion_radius,
            )
            return True
        if not self.leader_thermal_projection_filter_enabled or camera_point is None:
            return False
        return self._is_in_leader_projection(leader, camera_point, event_stamp, projection)

    def _is_in_leader_projection(self, leader, camera_point, event_stamp, projection=None):
        """Compare image directions without trusting the candidate's depth.

        For an optical-frame point, X/Z and Y/Z are normalized image pixels.
        Project the candidate ray to the leader's depth plane and measure its
        offset there. This is equivalent to a projected circular region, with
        pixel radii fx*r/Z and fy*r/Z, without requiring another CameraInfo.
        """
        candidate = camera_point.point
        if (not camera_point.header.frame_id or candidate.z <= 0.0
                or not all(math.isfinite(v) for v in (candidate.x, candidate.y, candidate.z))):
            return False
        # Leader coordinates are already in the fixed localization frame.
        # Use the camera pose at the thermal event time, not odometry receipt
        # time or the older matched leader stamp while UAV1 is moving.
        leader_at_event = PointStamped()
        leader_at_event.header.frame_id = self.localization_frame
        leader_at_event.header.stamp = rospy.Time.from_sec(event_stamp)
        leader_at_event.point = leader.point
        try:
            projected = self.tf_buffer.transform(
                leader_at_event, camera_point.header.frame_id, rospy.Duration(0.10)
            ).point
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "Cannot project leader into thermal candidate camera: %s", exc
            )
            return False
        if (projected.z <= 0.0
                or not all(math.isfinite(v) for v in (projected.x, projected.y, projected.z))):
            return False
        offset = math.hypot(
            candidate.x / candidate.z * projected.z - projected.x,
            candidate.y / candidate.z * projected.z - projected.y,
        )
        if projection is not None:
            projection["status"] = "clear" if offset > self.leader_thermal_projection_radius else "filtered"
        if offset > self.leader_thermal_projection_radius:
            return False
        rospy.logwarn_throttle(
            1.0, "Filtered UAV1 thermal candidate in UAV0 projection: "
            "offset=%.3fm radius=%.3fm candidate_depth=%.3fm leader_depth=%.3fm",
            offset, self.leader_thermal_projection_radius, candidate.z, projected.z,
        )
        return True

    def _is_ground_thermal_source(self, position):
        if not self.thermal_ground_filter_enabled:
            return False
        if position["z"] >= self.thermal_min_height:
            return False
        rospy.logwarn_throttle(
            1.0,
            "Filtered UAV1 ground thermal candidate: z=%.3fm min_height=%.3fm",
            position["z"],
            self.thermal_min_height,
        )
        return True

    def _retry_pending(self, _event):
        for source in ("color", "qr", "thermal"):
            self._try_process(source)
            self._try_save_unlocalized(source)

    def _wait_for_matching_image(self, source, stamp, tolerance=None):
        """Wait briefly for the debug image from the confirmed source frame.

        Detector callbacks publish the candidate/status and debug image on
        separate ROS topics.  The status can therefore reach this node first.
        Waiting here is bounded; if no same-frame image arrives, the caller
        records the confirmed JSON without attaching an unrelated image.
        """
        if tolerance is None:
            tolerance = self.image_tolerance
        deadline = time.monotonic() + self.image_wait_timeout
        while not rospy.is_shutdown():
            match = self.images.nearest_with_stamp(
                source, stamp, tolerance
            )
            if match is not None:
                image_stamp, image = match
                rospy.logdebug(
                    "Matched %s evidence image: event=%.6f image=%.6f delta=%.6f",
                    source,
                    stamp,
                    image_stamp,
                    abs(float(image_stamp) - float(stamp)),
                )
                return image
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)
        return None

    def _try_process(self, source):
        with self.lock:
            latest_point = self.latest_point.get(source)
            points = list(self.point_history.get(source, ()))
            status = self.latest_status.get(source)
            pair = (self.point_version.get(source, 0), self.status_version.get(source, 0))
        if latest_point is None or status is None:
            return
        result, confidence = status
        # Callback and timer retries share the cached status. Keep its stamp
        # and detector gate intact until this exact pair has been processed.
        result = dict(result)
        source_stamp = result.get("_source_stamp")
        point = latest_point
        event_stamp = point.header.stamp.to_sec()
        if source_stamp is not None:
            try:
                source_stamp = float(source_stamp)
            except (TypeError, ValueError):
                source_stamp = None
        if source_stamp is not None:
            matching_points = [
                item for item in points
                if abs(item.header.stamp.to_sec() - source_stamp)
                <= self.source_match_tolerance
            ]
            if not matching_points:
                # Do not pair a stable status with the newest point from a
                # different detector frame. The retry timer will process it
                # after the matching PointStamped callback arrives.
                return
            point = min(
                matching_points,
                key=lambda item: abs(item.header.stamp.to_sec() - source_stamp),
            )
            event_stamp = source_stamp
        if not point.header.frame_id:
            return
        with self.lock:
            previous = self.last_processed_pair.get(source, (0, 0))
            if pair[0] <= previous[0] or pair[1] <= previous[1]:
                return
        try:
            world = self.tf_buffer.transform(point, self.localization_frame, rospy.Duration(0.10))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "TF %s -> %s failed: %s", point.header.frame_id,
                                   self.localization_frame, exc)
            return
        d435_evidence = None
        if source == "thermal" and self.require_thermal_d435_evidence:
            # Image and candidate use independent ROS connections. Publishing
            # the image first does not guarantee that its callback runs first.
            d435_evidence = self._wait_for_matching_image(
                "thermal_d435", event_stamp,
                1e-7 if self.thermal_fused_topic else self.thermal_d435_ready_tolerance,
            )
            if d435_evidence is None:
                rospy.logwarn_throttle(
                    2.0, "Thermal candidate waiting for matching D435 evidence: stamp=%.6f",
                    event_stamp,
                )
                # Leave the pair pending for the existing retry timer. Missing
                # evidence must not reset accumulated thermal confirmation hits.
                return
        # Mark processed only after TF and required evidence succeed. Recheck
        # so callback and timer cannot process the same pair twice.
        with self.lock:
            previous = self.last_processed_pair.get(source, (0, 0))
            if pair[0] <= previous[0] or pair[1] <= previous[1]:
                return
            self.last_processed_pair[source] = pair
        position = {"x": world.point.x, "y": world.point.y, "z": world.point.z}
        target_type = {"color": "color_tag", "qr": "qr_code", "thermal": "thermal_source"}[source]
        if self._is_beyond_exit_plane(world):
            return
        if source == "thermal" and self._is_leader_thermal_source(position, event_stamp, point):
            return
        if source == "thermal" and self._is_ground_thermal_source(position):
            return
        # All three detectors publish raw candidates immediately, but each
        # detector also publishes its own stable/validated gate.  The
        # reporting layer must honor that gate uniformly; it must not turn a
        # repeated raw candidate into a confirmed target by adding another
        # unrelated frame counter.
        result.pop("_source_stamp", None)
        detector_stable = bool(result.pop("_detector_stable", True))
        detector_confirmable = bool(
            result.pop("_detector_confirmable", detector_stable)
        )
        result["detector_stable"] = detector_stable
        result["detector_confirmable"] = detector_confirmable
        with self.candidate_lock:
            candidate, confirmed = self.tracker.update(
                target_type,
                result,
                position,
                allow_confirmation=(detector_confirmable
                                    and self._recording_allowed(event_stamp)),
                timestamp=event_stamp,
            )
            candidate_number = candidate["local_number"]
            candidate_hits = candidate["hits"]
            if confirmed:
                self.seq += 1
                event_seq = self.seq
            else:
                event_seq = None
        observation = {
            "version": 1, "message_type": "observation", "target_type": target_type,
            "timestamp": event_stamp, "drone_id": self.drone_id,
            "source_timestamp": event_stamp,
            "target_id": "{}-{}-{:03d}".format(self.drone_id, target_type, candidate_number),
            "result": result,
            "position": {"frame_id": self.localization_frame, "unit": "m", **position},
            "hits": candidate_hits,
            "candidate": not confirmed,
            "confirmed": confirmed,
            "detector_stable": detector_stable,
            "detector_confirmable": detector_confirmable,
        }
        self.observation_pub.publish(String(data=json.dumps(observation, ensure_ascii=False)))
        # Candidates remain available on the local ROS observation topic for
        # RViz, but are never sent to Windows.  Confirmed remote data is
        # published below, after the same-frame stable image has been matched,
        # so JSON and evidence cannot get out of sync.
        if not confirmed:
            return
        report_position = self._world_to_channel(position)
        target_id = observation["target_id"]
        if source == "thermal":
            # All candidates share the raw thermal frame. The fusion status
            # retains its base stamp; candidate stamps differ by microseconds.
            thermal_stamp = result.get("source_timing", {}).get("thermal_stamp", event_stamp)
            image = self._wait_for_matching_image(source, thermal_stamp, 1e-7)
        else:
            image = self._wait_for_matching_image(source, event_stamp)
        image_id = "{}_seq{:06d}_{}.jpg".format(self.drone_id, event_seq, target_type)
        event = TargetEvent(
            seq=event_seq, timestamp=event_stamp, drone_id=self.drone_id,
            message_type="confirmed_target", target_id=target_id, target_type=target_type,
            result=result, position=report_position,
            confidence=confidence,
        ).to_dict()
        jpeg = None
        d435_jpeg = None
        if image is not None:
            if source == "thermal":
                image = draw_detection_overlay(image, event)
            jpeg = build_evidence_jpeg(image, event,
                                       (point.point.x, point.point.y, point.point.z))
            event["image"] = image_metadata(image_id, jpeg)
            self.store.save_image(image_id, jpeg, target_type=target_type)
        else:
            rospy.logwarn(
                "Confirmed %s %s has no same-frame debug image; "
                "skip image upload instead of using an unstable frame",
                target_type,
                target_id,
            )
        if source == "thermal":
            d435_image = d435_evidence
            if d435_image is None:
                d435_image = self._wait_for_matching_image(
                    "thermal_d435", event_stamp,
                    1e-7 if self.thermal_fused_topic else self.d435_image_tolerance
                )
            if d435_image is not None:
                d435_image_id = "{}_seq{:06d}_thermal_source_d435.jpg".format(
                    self.drone_id, event_seq
                )
                d435_jpeg = build_evidence_jpeg(
                    d435_image,
                    event,
                    (point.point.x, point.point.y, point.point.z),
                )
                event["d435_image"] = image_metadata(d435_image_id, d435_jpeg)
                self.store.save_image(d435_image_id, d435_jpeg, target_type=target_type)
            else:
                rospy.logwarn(
                    "Confirmed thermal target %s has no matching D435 debug image",
                    target_id,
                )
        # Only confirmed detector output is sent remotely.  This happens
        # after same-frame image matching, never before it.
        if self.enable_remote_upload:
            self.realtime_client.publish({**observation, "position": report_position.to_dict()})
        if self.store.append_event(event):
            if self.enable_remote_upload:
                self.json_client.enqueue(event)
                if jpeg is not None:
                    self.image_client.enqueue((image_id, jpeg))
                if d435_jpeg is not None:
                    self.image_client.enqueue((d435_image_id, d435_jpeg))
            rospy.loginfo("Confirmed %s %s at %s [%.3f %.3f %.3f]", target_type,
                          target_id, self.report_frame,
                          report_position.x, report_position.y, report_position.z)

    def _world_to_channel(self, position):
        """Convert a lidar-world point to the channel frame for final reporting.

        Rotation is a row-major 3x3 matrix; translation is the lidar-world
        origin expressed in channel coordinates, in metres.  Defaults reserve
        an identity transform until the measured calibration is supplied.
        """
        xyz = [position[axis] for axis in ("x", "y", "z")]
        rotation = self.world_to_channel_rotation
        translation = self.world_to_channel_translation
        converted = [
            sum(rotation[3 * row + col] * xyz[col] for col in range(3)) + translation[row]
            for row in range(3)
        ]
        return Position(self.report_frame, *converted)

    def shutdown(self):
        if self.retry_timer is not None:
            self.retry_timer.shutdown()
        if self.enable_remote_upload:
            self.json_client.stop()
            self.image_client.stop()
            self.realtime_client.stop()


def main():
    rospy.init_node("target_reporter")
    TargetReporterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
