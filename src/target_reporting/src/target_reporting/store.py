import json
import os
import tempfile
import threading
from datetime import datetime
from typing import Any, Dict, Optional

from .protocol import validate_event


class MissionStore:
    IMAGE_TARGET_TYPES = ("qr_code", "color_tag", "thermal_source")

    def __init__(self, root: str, drone_id: str, mission_id: Optional[str] = None):
        if mission_id is None:
            mission_id = datetime.now().strftime("mission_%Y%m%d_%H%M%S")
        self.root = os.path.abspath(os.path.expanduser(root))
        self.mission_dir = os.path.join(self.root, mission_id)
        self.image_dir = os.path.join(self.mission_dir, "images", "annotated")
        self.events_path = os.path.join(self.mission_dir, "reports.jsonl")
        self.event_acks_path = os.path.join(self.mission_dir, "event_acks.jsonl")
        self.image_acks_path = os.path.join(self.mission_dir, "image_acks.jsonl")
        self.drone_id = drone_id
        self._lock = threading.Lock()
        self._keys = set()
        self._event_acks = set()
        self._image_acks = set()
        os.makedirs(self.image_dir, exist_ok=True)
        self._load_keys()
        self._load_acks()

    def _load_keys(self) -> None:
        if not os.path.exists(self.events_path):
            return
        with open(self.events_path, "r", encoding="utf-8") as stream:
            for line in stream:
                if not line.strip():
                    continue
                event = json.loads(line)
                self._keys.add((str(event["drone_id"]), int(event["seq"])))

    def _load_acks(self) -> None:
        if os.path.exists(self.event_acks_path):
            with open(self.event_acks_path, "r", encoding="utf-8") as stream:
                self._event_acks.update(int(line.strip()) for line in stream if line.strip())
        if os.path.exists(self.image_acks_path):
            with open(self.image_acks_path, "r", encoding="utf-8") as stream:
                self._image_acks.update(line.strip() for line in stream if line.strip())

    @staticmethod
    def _append_ack(path: str, value: str) -> None:
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(value + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def append_event(self, event: Dict[str, Any]) -> bool:
        validate_event(event)
        key = (str(event["drone_id"]), int(event["seq"]))
        with self._lock:
            if key in self._keys:
                return False
            with open(self.events_path, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            self._keys.add(key)
            return True

    def has_event(self, drone_id: str, seq: int) -> bool:
        with self._lock:
            return (str(drone_id), int(seq)) in self._keys

    def mark_event_acked(self, seq: int) -> None:
        seq = int(seq)
        with self._lock:
            if seq in self._event_acks:
                return
            self._append_ack(self.event_acks_path, str(seq))
            self._event_acks.add(seq)

    def mark_image_acked(self, image_id: str) -> None:
        image_id = str(image_id)
        with self._lock:
            if image_id in self._image_acks:
                return
            self._append_ack(self.image_acks_path, image_id)
            self._image_acks.add(image_id)

    def _events(self):
        if not os.path.exists(self.events_path):
            return []
        with open(self.events_path, "r", encoding="utf-8") as stream:
            return [json.loads(line) for line in stream if line.strip()]

    def all_events(self):
        with self._lock:
            return self._events()

    def pending_events(self):
        with self._lock:
            acked = set(self._event_acks)
            events = self._events()
        return [event for event in events if int(event["seq"]) not in acked]

    def pending_images(self):
        with self._lock:
            acked = set(self._image_acks)
            events = self._events()
        pending = []
        for event in events:
            for field in ("image", "d435_image"):
                image = event.get(field) or {}
                image_id = image.get("id")
                if not image_id or image_id in acked:
                    continue
                target_type = event.get("target_type")
                path = self._image_path(image_id, target_type)
                if not os.path.exists(path):
                    # Records saved before type folders still use the flat layout.
                    path = self._image_path(image_id)
                if os.path.exists(path):
                    with open(path, "rb") as stream:
                        pending.append((image_id, stream.read()))
        return pending

    def _image_path(self, image_id: str, target_type: Optional[str] = None) -> str:
        safe_name = os.path.basename(image_id)
        if not safe_name or safe_name != image_id or not safe_name.lower().endswith(".jpg"):
            raise ValueError("invalid image id")
        if target_type is None:
            return os.path.join(self.image_dir, safe_name)
        if target_type not in self.IMAGE_TARGET_TYPES:
            raise ValueError("invalid image target type")
        return os.path.join(self.image_dir, target_type, safe_name)

    def save_image(
        self, image_id: str, jpeg_bytes: bytes, target_type: Optional[str] = None
    ) -> str:
        final_path = self._image_path(image_id, target_type)
        directory = os.path.dirname(final_path)
        os.makedirs(directory, exist_ok=True)
        fd, temp_path = tempfile.mkstemp(prefix=image_id + ".", suffix=".tmp", dir=directory)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(jpeg_bytes)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, final_path)
        except Exception:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
            raise
        return final_path
