#!/usr/bin/env python3
"""Record exact-frame crack detections from the existing ROS1 online detector."""
import argparse
import hashlib
import os
from pathlib import Path
import threading
import time

from crack_live_common import (DEFAULT_OUTPUT, ExactPairer, atomic_json, controls,
                               frame_key, read_json)
from crack_live_store import CaptureStore


def model_metadata(rospy):
    import rospkg
    settings = rospy.get_param('/yolo', {})
    reading = settings.get('model_reading', {})
    package = Path(rospkg.RosPack().get_path('yolo_detector'))
    hashes = {}
    for key in ('model_path', 'para_path'):
        path = Path(reading.get(key, ''))
        if not path.is_absolute():
            path = package / path
        if path.is_file():
            hashes[key] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    camera = {key: rospy.get_param('/' + key, None) for key in
              ('width', 'height', 'downsample', 'Offset_x', 'Offset_y', 'ExposureTime',
               'GainAuto', 'FrameRate', 'Gamma', 'GammaEnable', 'BalanceWhiteAuto')}
    mode = settings.get('detection_parameters', {}).get('mode', 'whole')
    return {'detector': {'settings': settings, 'files': hashes, 'mode': mode},
            'camera_requested_parameters': camera}


class RecorderNode:
    def __init__(self, rospy, output):
        from sensor_msgs.msg import Image
        from target_ros_msgs.msg import BoundingBoxes
        self.rospy = rospy
        self.root = Path(output).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        # One writer per output root; no PID guessing or stale-lock-file deletion.
        import fcntl
        self.lock_file = (self.root / '.recorder.lock').open('a')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.guard = threading.RLock()
        self.pairer = ExactPairer()
        self.config = controls(read_json(self.root / 'controls.json'))
        if not (self.root / 'controls.json').exists():
            atomic_json(self.root / 'controls.json', self.config)
        self.store = CaptureStore(self.root, model_metadata(rospy))
        self.latest = None
        self.last_pair_time = None
        self.last_image_time = None
        self.last_boxes_time = None
        self.invalid_messages = 0
        self.config_error = ''
        self.stopping = False
        self.image_topic = rospy.get_param('/subscribers/camera_reading/img_topic',
                                           '/hikcamera/img_stream')
        self.subscribers = [
            rospy.Subscriber(self.image_topic, Image, self.on_image,
                             queue_size=2, buff_size=2**27),
            rospy.Subscriber('/objects', BoundingBoxes, self.on_boxes, queue_size=20)
        ]
        self.last_status = 0.
        self.timer = rospy.Timer(rospy.Duration(.2), self.poll)
        rospy.on_shutdown(self.close)
        rospy.loginfo('Crack autosave ready: %s -> %s (exact image_header timestamp)',
                      self.image_topic, self.root)

    def consume(self, pair):
        if pair is not None:
            now = time.monotonic()
            self.latest = (pair, now)
            self.last_pair_time = time.time()
            self.store.submit(pair, self.config, now=now)

    def on_image(self, message):
        with self.guard:
            if self.stopping:
                return
            try:
                key = frame_key(message.header)
                self.last_image_time = time.time()
                self.consume(self.pairer.image(key, message, len(message.data)))
            except (ValueError, TypeError, AttributeError) as error:
                self.invalid_messages += 1
                self.rospy.logwarn_throttle(5, 'Crack recorder image rejected: %s', str(error))

    def on_boxes(self, message):
        with self.guard:
            if self.stopping:
                return
            try:
                key = frame_key(message.image_header)
                boxes = [{'class': b.Class, 'confidence': b.probability,
                          'xyxy': [b.xmin, b.ymin, b.xmax, b.ymax]}
                         for b in message.bounding_boxes]
                self.last_boxes_time = time.time()
                self.consume(self.pairer.result(key, boxes))
            except (ValueError, TypeError, AttributeError) as error:
                self.invalid_messages += 1
                self.rospy.logwarn_throttle(5, 'Crack recorder detections rejected: %s', str(error))

    def publish_status(self, running=True):
        status = {**self.store.snapshot(), 'running': running, 'updated_at': time.time(),
                  'pid': os.getpid(),
                  'last_image_at': self.last_image_time, 'last_boxes_at': self.last_boxes_time,
                  'last_matched_at': self.last_pair_time, 'config_error': self.config_error,
                  'cache_evictions': self.pairer.evictions, 'cache_bytes': self.pairer.bytes,
                  'invalid_messages': self.invalid_messages, 'image_topic': self.image_topic}
        atomic_json(self.root / 'status.json', status)

    def poll(self, _):
        with self.guard:
            if self.stopping:
                return
            try:
                value = read_json(self.root / 'controls.json')
                if value is None:
                    raise ValueError('采样条件文件无法读取，暂沿用上次有效设置。')
                self.config = controls(value)
                self.config_error = ''
            except ValueError as error:
                self.config_error = str(error)
            now = time.monotonic()
            self.pairer.expire(now)
            # A manual request may use the latest complete pair only if it is fresh.
            # Automatic/periodic saving is driven ONLY by new matched frames.
            request = self.config['manual_request']
            if (request and request['id'] != self.store.last_manual_id
                    and 0 <= time.time() - request['time'] <= 10
                    and self.latest and now - self.latest[1] <= 1.0):
                self.store.submit(self.latest[0], self.config, now=now)
            if now - self.last_status >= 1:
                self.last_status = now
                self.publish_status()
                error = self.store.snapshot()['last_error'] or self.config_error
                if error:
                    self.rospy.logerr_throttle(5, 'Crack autosave: %s', error)

    def close(self):
        with self.guard:
            if self.stopping:
                return
            self.stopping = True
            self.timer.shutdown()
            for subscriber in self.subscribers:
                subscriber.unregister()
        finished = self.store.close(timeout=5)
        self.publish_status(running=False)
        if not finished:
            self.rospy.logerr('Crack save worker did not finish within shutdown deadline.')
        self.lock_file.close()


def main():
    import rospy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('crack_capture_recorder', disable_signals=False)
    recorder = RecorderNode(rospy, args.output)
    try:
        rospy.spin()
    finally:
        recorder.close()


if __name__ == '__main__':
    main()
