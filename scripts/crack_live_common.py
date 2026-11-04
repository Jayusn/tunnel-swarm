"""Shared on-disk protocol and exact frame pairing; independent of ROS/Qt."""
from collections import OrderedDict
import json
import math
from pathlib import Path
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / 'captures/crack_live'
DEFAULT_CONTROLS = {
    'auto_save': True, 'min_interval_s': 2.0,
    'periodic_save': False, 'periodic_interval_s': 5.0,
    'distance_m': None, 'light': 'unknown', 'lux': None,
    'specimen_id': 'crack_01', 'notes': '', 'manual_request': None,
}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False),
                             encoding='utf-8')
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (FileNotFoundError, ValueError, OSError):
        return default


def controls(value):
    result = dict(DEFAULT_CONTROLS)
    if value is not None:
        if not isinstance(value, dict):
            raise ValueError('controls must be a JSON object')
        result.update({k: v for k, v in value.items() if k in result})
    for key in ('auto_save', 'periodic_save'):
        if type(result[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    for key, minimum in [('min_interval_s', 0), ('periodic_interval_s', 0.2)]:
        v = result[key]
        if type(v) not in (int, float) or not math.isfinite(v) or not minimum <= v <= 3600:
            raise ValueError('Invalid ' + key)
    for key in ('distance_m', 'lux'):
        v = result[key]
        if v is not None and (type(v) not in (int, float) or not math.isfinite(v) or
                              v < 0 or (key == 'distance_m' and v == 0)):
            raise ValueError('Invalid ' + key)
    if result['light'] not in ('unknown', 'bright', 'dark'):
        raise ValueError('Invalid light group')
    for key in ('specimen_id', 'notes'):
        if not isinstance(result[key], str) or len(result[key]) > 2000:
            raise ValueError('Invalid ' + key)
    request = result['manual_request']
    if request is not None:
        if (not isinstance(request, dict) or not isinstance(request.get('id'), str)
                or not 1 <= len(request['id']) <= 100
                or type(request.get('time')) not in (int, float)
                or not math.isfinite(request['time'])):
            raise ValueError('Invalid manual_request')
    return result


def inside(root, relative):
    """Resolve data-relative paths without allowing a corrupt index to escape."""
    root = Path(root).resolve()
    target = (root / relative).resolve()
    if target != root and root not in target.parents:
        raise ValueError('Path is outside capture directory')
    return target


def frame_key(header):
    secs, nsecs = int(header.stamp.secs), int(header.stamp.nsecs)
    if secs < 0 or not 0 <= nsecs < 1000000000 or (secs == 0 and nsecs == 0):
        raise ValueError('Image requires a nonzero valid ROS timestamp')
    return secs, nsecs, str(header.frame_id)


class ExactPairer:
    """Bound raw-message memory; accept either callback order, never approximate."""
    def __init__(self, max_frames=16, max_bytes=256 * 1024 * 1024, max_age=3.0):
        self.images, self.results = OrderedDict(), OrderedDict()
        self.max_frames, self.max_bytes, self.max_age = max_frames, max_bytes, max_age
        self.bytes = 0
        self.evictions = 0
        self.completed = OrderedDict()

    def expire(self, now):
        for key, (_, _, stamp) in list(self.images.items()):
            if now - stamp > self.max_age:
                self.bytes -= self.images.pop(key)[1]
                self.evictions += 1
        for key, (_, stamp) in list(self.results.items()):
            if now - stamp > self.max_age:
                self.results.pop(key)
                self.evictions += 1
        while len(self.images) > self.max_frames or self.bytes > self.max_bytes:
            _, (_, size, _) = self.images.popitem(last=False)
            self.bytes -= size
            self.evictions += 1
        while len(self.results) > self.max_frames * 2:
            self.results.popitem(last=False)
            self.evictions += 1
        while len(self.completed) > 256:
            self.completed.popitem(last=False)

    def _match(self, key, now):
        if key in self.images and key in self.results:
            image, size, _ = self.images.pop(key)
            self.bytes -= size
            result, _ = self.results.pop(key)
            self.completed[key] = now
            return key, image, result
        return None

    def image(self, key, value, size, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        if key in self.completed:
            return None
        if key in self.images:
            self.bytes -= self.images.pop(key)[1]
        self.images[key] = (value, size, now)
        self.bytes += size
        self.expire(now)
        return self._match(key, now)

    def result(self, key, value, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        if key in self.completed:
            return None
        self.results[key] = (value, now)
        self.expire(now)
        return self._match(key, now)
