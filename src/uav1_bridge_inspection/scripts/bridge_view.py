#!/usr/bin/env python3
"""左侧检测图像，右侧嵌入 RViz 地图；只订阅，不发送控制指令。"""
import sys
import threading

import rospy
from rviz import bindings as rviz
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QApplication, QLabel, QMainWindow, QSizePolicy, QSplitter, QVBoxLayout, QWidget
from sensor_msgs.msg import Image


class View(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('UAV1 桥梁巡检 — 检测 / 建图')
        self.resize(1600, 950)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames, self.labels, self.subs = {}, {}, []
        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        layout = QVBoxLayout(left)
        for title, topic in [('裂缝', '/UAV1/bridge/crack/debug_image'),
                             ('热源', '/UAV1/thermal/d435_debug_image'),
                             ('二维码', '/UAV1/vision/qr_debug_image'),
                             ('18 cm ArUco 降落', '/UAV1/landing/debug_image')]:
            layout.addWidget(QLabel(title))
            label = QLabel('等待图像')
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(160, 100)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
            label.setStyleSheet('background: #181b20; color: #dddddd')
            layout.addWidget(label, 1)
            self.labels[topic] = label
            self.subs.append(rospy.Subscriber(topic, Image, self.image_cb,
                                             callback_args=topic, queue_size=1, buff_size=8388608))
        self.rviz = rviz.VisualizationFrame()
        self.rviz.setSplashPath('')
        self.rviz.initialize()
        reader, config = rviz.YamlConfigReader(), rviz.Config()
        reader.readFile(config, rospy.get_param('~rviz_config'))
        self.rviz.load(config)
        self.rviz.setMenuBar(None)
        self.rviz.setStatusBar(None)
        self.rviz.setHideButtonVisibility(False)
        split.addWidget(left)
        split.addWidget(self.rviz)
        split.setSizes([500, 1100])
        self.setCentralWidget(split)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_images)
        self.timer.start(100)

    def image_cb(self, msg, topic):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
            with self.lock:
                self.frames[topic] = image
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5, '显示图像转换失败: %s', exc)

    def update_images(self):
        if rospy.is_shutdown():
            self.close()
            return
        with self.lock:
            for topic, frame in self.frames.items():
                label = self.labels[topic]
                label.setPixmap(QPixmap.fromImage(frame).scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def closeEvent(self, event):
        rospy.signal_shutdown('关闭巡检显示')
        event.accept()


if __name__ == '__main__':
    rospy.init_node('bridge_view', disable_signals=True)
    app = QApplication(sys.argv)
    view = View()
    view.show()
    sys.exit(app.exec_())
