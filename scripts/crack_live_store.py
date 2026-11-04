"""Asynchronous lossless capture storage. No inference, ROS, or GUI dependency."""
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import queue
import re
import shutil
import threading
import time
import uuid

import cv2
import numpy as np

from crack_live_common import atomic_json, controls, inside, read_json
from crack_capture_catalog import append_capture


def named_capture(root, stamp_s, stamp_ns, condition, reason):
    """Create a readable date/group directory, without overwriting another capture."""
    captured = datetime.fromtimestamp(stamp_s, timezone.utc).astimezone()
    specimen = re.sub(r'[^\w.-]+', '_', condition['specimen_id']).strip('._')[:24] or 'crack'
    distance = ('{:g}m'.format(condition['distance_m'])
                if condition['distance_m'] is not None else '距离未设')
    light = {'bright': '明', 'dark': '暗'}.get(condition['light'], '光照未设')
    trigger = {'detection': '自动', 'manual': '手动', 'periodic': '定时'}[reason]
    stem = '{}_{:03d}_{}_{}_{}_{}'.format(
        captured.strftime('%H-%M-%S'), stamp_ns // 1000000, specimen, distance, light, trigger)
    parent = inside(root, captured.strftime('%Y-%m-%d'))
    parent.mkdir(parents=True, exist_ok=True)
    suffix = 1
    while True:
        folder = parent / (stem if suffix == 1 else '{}_{:02d}'.format(stem, suffix))
        try:
            folder.mkdir()
            break
        except FileExistsError:
            suffix += 1
    return folder, captured.replace(microsecond=stamp_ns // 1000).isoformat()


def image_to_bgr(message):
    """Decode only selected ROS frames. Honor row padding; do not rescale."""
    width, height = int(message.width), int(message.height)
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'bgra8': 4, 'rgba8': 4}
    if message.encoding not in channels or width < 1 or height < 1:
        raise ValueError('Unsupported camera image encoding or size: ' + message.encoding)
    count = channels[message.encoding]
    step = int(message.step)
    if step < width * count or len(message.data) < height * step:
        raise ValueError('Invalid image step/data length')
    raw = np.frombuffer(message.data, np.uint8, count=height * step).reshape(height, step)
    pixels = raw[:, :width * count].reshape(height, width, count)
    conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
                   'bgra8': cv2.COLOR_BGRA2BGR, 'rgba8': cv2.COLOR_RGBA2BGR}
    if message.encoding in conversions:
        pixels = cv2.cvtColor(pixels[:, :, 0] if count == 1 else pixels,
                              conversions[message.encoding])
    return np.ascontiguousarray(pixels)


def crack_boxes(boxes, width, height):
    output = []
    for box in boxes:
        if box.get('class') != 'crack':
            continue
        score = float(box['confidence'])
        coordinates = box['xyxy']
        if not math.isfinite(score) or not 0 <= score <= 1 or len(coordinates) != 4:
            continue
        if not all(math.isfinite(float(x)) for x in coordinates):
            continue
        x1, y1, x2, y2 = map(int, coordinates)
        xyxy = [max(0, x1), max(0, y1), min(width, x2), min(height, y2)]
        if xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]:
            continue
        output.append({'id': len(output), 'class': 'crack', 'confidence': score, 'xyxy': xyxy})
    return output


def write_png(path, image):
    ok, buffer = cv2.imencode('.png', image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise IOError('PNG encoding failed: ' + str(path))
    data = buffer.tobytes()
    Path(path).write_bytes(data)
    return hashlib.sha256(data).hexdigest()


class CaptureStore:
    def __init__(self, root, session_metadata, min_free_bytes=1024**3):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.session = 'session_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
        self.min_free_bytes = min_free_bytes
        self.session_metadata = dict(session_metadata)
        self.session_created_at = datetime.now(timezone.utc).isoformat()
        self.jobs = queue.Queue(maxsize=2)
        self.lock = threading.Lock()
        self.accepting = True
        self.last_auto = float('-inf')
        self.last_periodic = float('-inf')
        self.last_manual_id = None
        self.stats = {'saved': 0, 'queue_dropped': 0, 'write_errors': 0,
                      'last_error': '', 'last_saved': None, 'last_manual_saved': None}
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self._worker, name='crack-png-writer', daemon=True)
        self.thread.start()

    def submit(self, pair, conditions, now=None, wall_time=None):
        now = time.monotonic() if now is None else now
        wall_time = time.time() if wall_time is None else wall_time
        condition = controls(conditions)
        key, message, result = pair
        boxes = crack_boxes(result, int(message.width), int(message.height))
        request = condition['manual_request']
        manual = (request is not None and request['id'] != self.last_manual_id
                  and 0 <= wall_time - request['time'] <= 10)
        with self.lock:
            reason = ('manual' if manual else
                      'detection' if boxes and condition['auto_save'] and
                      now - self.last_auto >= condition['min_interval_s'] else
                      'periodic' if condition['periodic_save'] and
                      now - self.last_periodic >= condition['periodic_interval_s'] else None)
            if not self.accepting or reason is None:
                return False
            job = (key, message, boxes, dict(condition), reason)
            try:
                self.jobs.put_nowait(job)
            except queue.Full:
                self.stats['queue_dropped'] += 1
                return False
            if reason == 'manual':
                self.last_manual_id = request['id']
            if boxes:
                self.last_auto = now
            self.last_periodic = now
            return True

    def snapshot(self):
        with self.lock:
            return {**self.stats, 'queued': self.jobs.qsize(), 'session': self.session}

    def save_capture(self, job):
        key, message, boxes, condition, reason = job
        if shutil.disk_usage(self.root).free < self.min_free_bytes:
            raise IOError('可用磁盘空间不足1 GiB，已停止本次写图；不会删除旧照片。')
        image = image_to_bgr(message)
        stamp_s, stamp_ns, frame_id = key
        folder, captured_at = named_capture(self.root, stamp_s, stamp_ns, condition, reason)
        name = folder.name
        original = name + '_原图.png'
        annotated_name = name + '_检测图.png'
        original_hash = write_png(folder / original, image)
        annotated = image.copy()
        scale = max(1.0, min(image.shape[:2]) / 640.0)
        for box in boxes:
            x1, y1, x2, y2 = box['xyxy']
            crop = '{}_裂缝{:03d}.png'.format(name, box['id'] + 1)
            write_png(folder / crop, image[y1:y2, x1:x2])
            box['crop'] = crop
            cv2.rectangle(annotated, (x1, y1), (x2 - 1, y2 - 1), (0, 220, 255),
                          max(2, round(2 * scale)))
            label = 'crack {:.1f}%'.format(box['confidence'] * 100)
            font = .8 * scale
            thickness = max(1, round(2 * scale))
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font, thickness)
            tx = min(x1, max(0, image.shape[1] - tw - 8))
            ty = max(th + 5, y1)
            cv2.rectangle(annotated, (tx, max(0, ty - th - 5)),
                          (min(image.shape[1] - 1, tx + tw + 6), min(image.shape[0] - 1, ty + baseline + 3)),
                          (16, 16, 16), -1)
            cv2.putText(annotated, label, (tx + 3, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        font, (0, 220, 255), thickness, cv2.LINE_AA)
        write_png(folder / annotated_name, annotated)
        metadata = {
            'schema_version': 1, 'created_at': datetime.now(timezone.utc).isoformat(),
            'storage_layout': 'date_named_capture_v2', 'capture_name': name,
            'captured_at': captured_at, 'session_id': self.session,
            'session_created_at': self.session_created_at,
            'ros_stamp': {'secs': stamp_s, 'nsecs': stamp_ns}, 'frame_id': frame_id,
            'image_seq': int(message.header.seq), 'source_encoding': message.encoding,
            'size': [int(message.width), int(message.height)], 'reason': reason,
            'conditions': condition, 'boxes': boxes, 'original': original,
            'annotated': annotated_name, 'original_sha256': original_hash,
            'detector': self.session_metadata.get('detector', {}),
            'camera_requested_parameters': self.session_metadata.get('camera_requested_parameters', {}),
            'note': '同时间戳原图与检测框；原图未缩放。带框图根据该帧框重新绘制。相机参数为配置请求值。'
        }
        # Metadata is the commit marker: viewers never load partially written events.
        atomic_json(folder / 'capture.json', metadata)
        relative = str(folder.relative_to(self.root)).replace('\\', '/')
        item = {'path': relative, 'created_at': metadata['created_at'], 'size': metadata['size'],
                'display_name': name, 'captured_at': captured_at, 'specimen_id': condition['specimen_id'],
                'boxes': len(boxes), 'reason': reason, 'distance_m': condition['distance_m'],
                'light': condition['light']}
        append_capture(self.root, item)
        return relative

    def _worker(self):
        try:
            while self.accepting or not self.jobs.empty():
                try:
                    job = self.jobs.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    relative = self.save_capture(job)
                    with self.lock:
                        self.stats['saved'] += 1
                        self.stats['last_saved'] = relative
                        self.stats['last_error'] = ''
                        if job[4] == 'manual':
                            self.stats['last_manual_saved'] = job[3]['manual_request']['id']
                except Exception as error:
                    with self.lock:
                        self.stats['write_errors'] += 1
                        self.stats['last_error'] = str(error)
                finally:
                    self.jobs.task_done()
        finally:
            self.finished.set()

    def close(self, timeout=5):
        with self.lock:
            self.accepting = False
        return self.finished.wait(timeout)
