#!/usr/bin/env python3
"""NUC-local Qt viewer for immutable captured originals and pixel-width inspection."""
import argparse
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import signal
import sys
import time
import uuid

from PyQt5 import QtCore, QtGui, QtWidgets
from crack_live_common import DEFAULT_OUTPUT, atomic_json, controls, inside, read_json
from crack_capture_catalog import (capture_folder, catalog_lock, checked_metadata,
                                   delete_capture, entry_from_metadata, event_order,
                                   rename_capture, scan_captures)

DISTANCE_PRESETS = (0.5, 0.8) + tuple(round(1 + i / 10, 1) for i in range(11)) + (2.5, 3.0)


def distance_from_text(text):
    text = text.strip()
    if text in ('', '距离未设置', '未设置'):
        return None
    if text.lower().endswith('m') or text.endswith('米'):
        text = text[:-1].strip()
    try:
        value = float(text)
    except ValueError:
        raise ValueError('距离请填写米数，例如1.1、1.35、1.75；留空表示未设置。')
    if not math.isfinite(value) or not 0.001 <= value <= 1000:
        raise ValueError('距离范围为0.001～1000米；留空表示未设置。')
    if abs(value - round(value, 3)) > 1e-9:
        raise ValueError('距离最多保留3位小数，例如1.375米。')
    return round(value, 3)


def saved_name(event):
    name = event.get('display_name') or event.get('capture_name')
    if name:
        return name
    try:
        stamp = event.get('captured_at') or event['created_at']
        return datetime.fromisoformat(stamp.replace('Z', '+00:00')).astimezone().strftime('%Y-%m-%d %H:%M:%S')
    except (ValueError, KeyError, TypeError):
        return event.get('path', '已保存照片').split('/')[-1]


class PixelView(QtWidgets.QGraphicsView):
    measured = QtCore.pyqtSignal(object)
    interacted = QtCore.pyqtSignal()
    previous_requested = QtCore.pyqtSignal()
    next_requested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setScene(QtWidgets.QGraphicsScene(self))
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        self.setBackgroundBrush(QtGui.QColor('#151a23'))
        self.setTransformationAnchor(self.NoAnchor)
        self.setResizeAnchor(self.AnchorViewCenter)
        self.setMouseTracking(True)
        self.size = (0, 0)
        self.points = []
        self.sections = []
        self.boxes = []
        self.drag = None
        self.press = None
        self.moved = False
        self.show_boxes = True
        self.setMinimumSize(420, 300)
        self.photo_previous = self.photo_button('‹', '上一张；第一张会回到最后一张。')
        self.photo_next = self.photo_button('›', '下一张；最后一张会回到第一张。')
        self.photo_previous.clicked.connect(lambda: self.previous_requested.emit())
        self.photo_next.clicked.connect(lambda: self.next_requested.emit())
        self.viewport().installEventFilter(self)
        self.position_photo_buttons()

    def photo_button(self, text, tooltip):
        # QWidget.scroll() moves viewport children with the image. Keep these
        # controls on the outer view so zooming/panning cannot displace them.
        button = QtWidgets.QToolButton(self)
        button.setText(text)
        button.setToolTip(tooltip)
        button.setFocusPolicy(QtCore.Qt.NoFocus)
        button.setCursor(QtCore.Qt.PointingHandCursor)
        button.setMouseTracking(True)
        button.setAttribute(QtCore.Qt.WA_Hover, True)
        button.setStyleSheet(
            'QToolButton { background: transparent; color: transparent; border: none; '
            'font-size: 36px; font-weight: bold; } '
            'QToolButton:hover { background: rgba(25, 45, 65, 150); color: white; } '
            'QToolButton:pressed { background: rgba(30, 110, 170, 210); color: white; }')
        button.setEnabled(False)
        button.hide()
        return button

    def position_photo_buttons(self):
        area = self.viewport().geometry()
        width = min(88, max(56, area.width() // 14))
        self.photo_previous.setGeometry(area.left(), area.top(), width, area.height())
        self.photo_next.setGeometry(area.right() - width + 1, area.top(), width, area.height())
        self.photo_previous.raise_()
        self.photo_next.raise_()

    def eventFilter(self, watched, event):
        if watched is self.viewport() and event.type() in (QtCore.QEvent.Resize, QtCore.QEvent.Move):
            self.position_photo_buttons()
        return super().eventFilter(watched, event)

    def set_photo_navigation(self, enabled):
        for button in (self.photo_previous, self.photo_next):
            button.setEnabled(enabled)
            button.setVisible(enabled)

    def set_image(self, image, boxes):
        self.scene().clear()
        self.scene().addPixmap(QtGui.QPixmap.fromImage(image))
        self.size = (image.width(), image.height())
        self.setSceneRect(0, 0, *self.size)
        self.points = []
        self.sections = []
        self.boxes = boxes
        self.viewport().update()

    def inspection_area(self, box=None, padding=0.2):
        """Viewport bounds only; source pixels, box and measurement coordinates stay intact."""
        bounds = QtCore.QRectF(0, 0, *self.size)
        if box is None:
            return bounds
        area = QtCore.QRectF(box[0], box[1], box[2] - box[0], box[3] - box[1])
        dx, dy = area.width() * padding, area.height() * padding
        return area.adjusted(-dx, -dy, dx, dy).intersected(bounds)

    def fit(self, box=None, padding=0.2):
        if self.size[0] == 0:
            return
        area = self.inspection_area(box, padding)
        self.fitInView(area, QtCore.Qt.KeepAspectRatio)

    def zoom_to(self, value):
        if not self.size[0]:
            return
        self.interacted.emit()
        center = self.mapToScene(self.viewport().rect().center())
        self.resetTransform()
        self.scale(value, value)
        self.centerOn(center)
        self.viewport().update()

    def wheelEvent(self, event):
        if not self.size[0]:
            return
        self.interacted.emit()
        before = self.mapToScene(event.pos())
        scale = self.transform().m11()
        target = min(64., max(.03125, scale * (2 if event.angleDelta().y() > 0 else .5)))
        self.scale(target / scale, target / scale)
        after = self.mapToScene(event.pos())
        self.translate(after.x() - before.x(), after.y() - before.y())
        self.viewport().update()
        event.accept()

    def mousePressEvent(self, event):
        self.press = event.pos()
        self.moved = False
        if event.button() in (QtCore.Qt.RightButton, QtCore.Qt.MiddleButton) or (
                event.modifiers() & QtCore.Qt.ShiftModifier):
            self.interacted.emit()
            self.drag = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self, event):
        if self.press and (event.pos() - self.press).manhattanLength() > 4:
            self.moved = True
        if self.drag is not None:
            delta = event.pos() - self.drag
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            self.drag = event.pos()
        event.accept()

    def mouseReleaseEvent(self, event):
        if self.drag is not None:
            self.drag = None
            self.unsetCursor()
            return
        if event.button() != QtCore.Qt.LeftButton or self.moved or self.transform().m11() < 8:
            return
        p = self.mapToScene(event.pos())
        point = [int(math.floor(p.x() + .5)), int(math.floor(p.y() + .5))]
        if 0 <= point[0] <= self.size[0] and 0 <= point[1] <= self.size[1]:
            self.interacted.emit()
            self.points = self.points + [point] if len(self.points) == 1 else [point]
            self.viewport().update()
            self.measured.emit(self.points)

    def drawForeground(self, painter, rectangle):
        if not self.size[0]:
            return
        painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
        if self.transform().m11() >= 8:
            visible = rectangle.intersected(QtCore.QRectF(0, 0, *self.size))
            pen = QtGui.QPen(QtGui.QColor(145, 195, 255, 110))
            pen.setCosmetic(True)
            painter.setPen(pen)
            for x in range(max(0, math.floor(visible.left())), min(self.size[0], math.ceil(visible.right())) + 1):
                painter.drawLine(QtCore.QPointF(x, visible.top()), QtCore.QPointF(x, visible.bottom()))
            for y in range(max(0, math.floor(visible.top())), min(self.size[1], math.ceil(visible.bottom())) + 1):
                painter.drawLine(QtCore.QPointF(visible.left(), y), QtCore.QPointF(visible.right(), y))
        if self.show_boxes:
            pen = QtGui.QPen(QtGui.QColor('#ffc063'), 2)
            pen.setCosmetic(True)
            painter.setPen(pen)
            for box in self.boxes:
                x1, y1, x2, y2 = box['xyxy']
                painter.drawRect(QtCore.QRectF(x1, y1, x2 - x1, y2 - y1))
        pen = QtGui.QPen(QtGui.QColor('#40e3ff'), 2)
        for section in self.sections:
            points = section['endpoints']
            saved_pen = QtGui.QPen(QtGui.QColor('#6aff88'), 2)
            saved_pen.setCosmetic(True)
            painter.setPen(saved_pen)
            painter.drawLine(QtCore.QPointF(*points[0]), QtCore.QPointF(*points[1]))
            position = painter.worldTransform().map(QtCore.QPointF(*points[1]))
            painter.save()
            painter.resetTransform()
            painter.setFont(QtGui.QFont(painter.font().family(), 10))
            painter.drawText(position + QtCore.QPointF(7, -7), '{}: {:.2f}px'.format(
                section['label'], section['width_px']))
            painter.restore()
        pen.setCosmetic(True)
        painter.setPen(pen)
        if len(self.points) == 2:
            painter.drawLine(QtCore.QPointF(*self.points[0]), QtCore.QPointF(*self.points[1]))
        radius = 4 / self.transform().m11()
        painter.setBrush(QtGui.QColor('#40e3ff'))
        for p in self.points:
            painter.drawEllipse(QtCore.QPointF(*p), radius, radius)


class HistoryLoader(QtCore.QThread):
    loaded = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, root, parent):
        super().__init__(parent)
        self.root = root

    def run(self):
        try:
            events = scan_captures(self.root, self.isInterruptionRequested)
            if not self.isInterruptionRequested():
                self.loaded.emit(events)
        except (OSError, ValueError) as error:
            self.failed.emit(str(error))


class Window(QtWidgets.QMainWindow):
    def __init__(self, root):
        super().__init__()
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.setWindowTitle('裂缝自动保存与原图像素查看')
        self.resize(1150, 850)
        self.current_path = None
        self.metadata = None
        self.loaded_hash = None
        self.events_signature = None
        self.catalog_signature = None
        self.known_entries = {}
        self.entries = []
        self.history_ready = False
        self.manual_id = None
        self.manual_at = 0
        self.preferred_box = 0
        self.build_ui()
        self.load_controls()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(700)
        self.refresh()
        self.history_loader = HistoryLoader(self.root, self)
        self.history_loader.loaded.connect(self.history_loaded)
        self.history_loader.failed.connect(self.history_failed)
        self.history_loader.start()
        QtWidgets.QApplication.instance().aboutToQuit.connect(self.stop_history)

    def build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.status = QtWidgets.QLabel('等待在线采集节点；可查看已保存的历史图片。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        conditions_row = QtWidgets.QHBoxLayout()
        self.distance = QtWidgets.QComboBox()
        self.distance.setEditable(True)
        self.distance.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
        self.distance.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.distance.setMinimumContentsLength(8)
        self.distance.setToolTip('可直接输入0.001～1000米，最多3位小数；留空表示未设置。点“应用条件”后用于后续保存。')
        self.distance.addItem('距离未设置', None)
        for d in DISTANCE_PRESETS:
            self.distance.addItem('{:g} m'.format(d), d)
        self.light = QtWidgets.QComboBox()
        for label, value in [('光照未设置', 'unknown'), ('明', 'bright'), ('暗', 'dark')]:
            self.light.addItem(label, value)
        self.lux = QtWidgets.QLineEdit()
        self.lux.setPlaceholderText('照度lux，可空')
        self.lux.setMaximumWidth(130)
        self.specimen = QtWidgets.QLineEdit()
        self.specimen.setMaximumWidth(140)
        conditions_row.addWidget(QtWidgets.QLabel('采样距离'))
        for widget in (self.distance, QtWidgets.QLabel('光照'), self.light, self.lux, QtWidgets.QLabel('裂缝编号'), self.specimen):
            conditions_row.addWidget(widget)
        apply = QtWidgets.QPushButton('应用条件')
        apply.setToolTip('保存距离、光照等设置，用于之后新采集的照片及命名；已保存的照片请用“补录本组实验标签”。')
        apply.clicked.connect(lambda: self.apply_controls())
        conditions_row.addWidget(apply)
        layout.addLayout(conditions_row)
        options = QtWidgets.QHBoxLayout()
        self.auto = QtWidgets.QCheckBox('检出自动保存')
        self.interval = QtWidgets.QDoubleSpinBox()
        self.interval.setRange(0, 3600)
        self.interval.setDecimals(1)
        self.interval.setSuffix(' 秒/组')
        self.interval.setToolTip('0表示每个匹配帧都尝试保存；队列满时计入状态中的队列丢弃。')
        self.periodic = QtWidgets.QCheckBox('另存无框对照')
        self.periodic.setToolTip('按固定间隔保存匹配帧，包含没有框的图，用于记录漏检。')
        self.period = QtWidgets.QDoubleSpinBox()
        self.period.setRange(.2, 3600)
        self.period.setDecimals(1)
        self.period.setSuffix(' 秒')
        for widget in (self.auto, self.interval, self.periodic, self.period):
            options.addWidget(widget)
        self.manual = QtWidgets.QPushButton('保存当前帧（含无框）')
        self.manual.clicked.connect(self.request_frame)
        options.addWidget(self.manual)
        layout.addLayout(options)
        layout.addWidget(QtWidgets.QLabel('修改设置后点“应用条件”。明暗、距离为手工标记，程序不会自动调整相机曝光或增益。'))
        navigation = QtWidgets.QHBoxLayout()
        self.previous_button = QtWidgets.QPushButton('上一张')
        self.previous_button.setShortcut('Left')
        self.previous_button.setToolTip('查看上一组（←）；第一组回到最后一组，并暂停跟随最新图片。')
        self.previous_button.clicked.connect(lambda: self.navigate(-1))
        navigation.addWidget(self.previous_button)
        self.next_button = QtWidgets.QPushButton('下一张')
        self.next_button.setShortcut('Right')
        self.next_button.setToolTip('查看下一组（→）；最后一组回到第一组，并暂停跟随最新图片。')
        self.next_button.clicked.connect(lambda: self.navigate(1))
        navigation.addWidget(self.next_button)
        self.position = QtWidgets.QLabel('0 / 0 组')
        navigation.addWidget(self.position)
        navigation.addStretch(1)
        self.rename_button = QtWidgets.QPushButton('重命名本组')
        self.rename_button.setToolTip('修改当前整组文件夹和配套图片名称，保留检测信息与像素测量。')
        self.rename_button.setEnabled(False)
        self.rename_button.clicked.connect(self.rename_current)
        navigation.addWidget(self.rename_button)
        self.delete_button = QtWidgets.QPushButton('删除本组（Del）')
        self.delete_button.setStyleSheet('QPushButton { color: #c0392b; }')
        self.delete_button.setToolTip('按Delete或点击此按钮，直接永久删除当前整组：原图、检测图、全部裂缝裁剪和测量记录。')
        self.delete_button.setEnabled(False)
        self.delete_button.clicked.connect(self.delete_current)
        self.delete_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), self)
        self.delete_shortcut.setContext(QtCore.Qt.WindowShortcut)
        self.delete_shortcut.setAutoRepeat(False)
        self.delete_shortcut.activated.connect(self.delete_from_key)
        navigation.addWidget(self.delete_button)
        layout.addLayout(navigation)
        row = QtWidgets.QHBoxLayout()
        self.events = QtWidgets.QComboBox()
        self.events.setMinimumWidth(250)
        self.events.currentIndexChanged.connect(self.select_event)
        row.addWidget(self.events, 1)
        self.follow = QtWidgets.QCheckBox('跟随最新保存图片')
        self.follow.setChecked(True)
        self.follow.toggled.connect(self.follow_changed)
        row.addWidget(self.follow)
        history = QtWidgets.QPushButton('打开保存目录')
        history.clicked.connect(lambda: QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(self.root))))
        row.addWidget(history)
        self.open_group = QtWidgets.QPushButton('打开本组')
        self.open_group.setEnabled(False)
        self.open_group.clicked.connect(self.open_current_folder)
        row.addWidget(self.open_group)
        layout.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        self.box = QtWidgets.QComboBox()
        self.box.currentIndexChanged.connect(self.select_box)
        self.box.activated.connect(self.box_activated)
        row.addWidget(self.box, 1)
        for name, zoom in [('8倍', 8), ('16倍', 16), ('32倍', 32)]:
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _, z=zoom: self.view.zoom_to(z))
            row.addWidget(button)
        fit = QtWidgets.QPushButton('适合区域')
        fit.clicked.connect(lambda: self.fit_selected())
        row.addWidget(fit)
        self.context_margin = QtWidgets.QComboBox()
        for percent in (0, 20, 50, 100):
            self.context_margin.addItem('周边 {}%'.format(percent), percent / 100.)
        self.context_margin.setCurrentIndex(1)
        self.context_margin.setToolTip('在检测框四周各多显示框宽/高的相应比例；仅调整查看范围。'
                                       '不修改检测框或原图像素。右键拖动可继续查看框外。')
        self.context_margin.currentIndexChanged.connect(lambda _: self.fit_selected())
        row.addWidget(self.context_margin)
        self.overlay = QtWidgets.QCheckBox('显示框')
        self.overlay.setChecked(True)
        self.overlay.toggled.connect(self.overlay_changed)
        row.addWidget(self.overlay)
        clear = QtWidgets.QPushButton('清除截面')
        clear.clicked.connect(self.clear_measurement)
        row.addWidget(clear)
        layout.addLayout(row)
        self.view = PixelView()
        self.view.interacted.connect(lambda: self.follow.setChecked(False))
        self.view.measured.connect(self.save_measurement)
        self.view.previous_requested.connect(lambda: self.navigate(-1))
        self.view.next_requested.connect(lambda: self.navigate(1))
        layout.addWidget(self.view, 1)
        self.measurement = QtWidgets.QLabel('原图网格每格＝1像素。放大至8倍以上，沿垂直裂缝走向的方向点两侧边界。')
        self.measurement.setWordWrap(True)
        layout.addWidget(self.measurement)
        self.info = QtWidgets.QLabel('滚轮缩放；右键或Shift＋左键拖动。开始测量会暂停跟随，图片不会中途替换。')
        self.info.setWordWrap(True)
        layout.addWidget(self.info)
        self.setCentralWidget(central)

    def set_distance_value(self, value):
        index = self.distance.findData(value)
        if index < 0:
            self.distance.addItem('{:g} m'.format(value), value)
            index = self.distance.count() - 1
        self.distance.setCurrentIndex(index)

    def load_controls(self):
        try:
            c = controls(read_json(self.root / 'controls.json'))
            self.set_distance_value(c['distance_m'])
            self.light.setCurrentIndex(max(0, self.light.findData(c['light'])))
            self.lux.setText('' if c['lux'] is None else str(c['lux']))
            self.specimen.setText(c['specimen_id'])
            self.auto.setChecked(c['auto_save'])
            self.interval.setValue(c['min_interval_s'])
            self.periodic.setChecked(c['periodic_save'])
            self.period.setValue(c['periodic_interval_s'])
        except ValueError as error:
            self.info.setText(str(error))

    def apply_controls(self, request=None):
        try:
            c = controls(read_json(self.root / 'controls.json'))
            c.update(distance_m=distance_from_text(self.distance.currentText()), light=self.light.currentData(),
                     lux=float(self.lux.text()) if self.lux.text().strip() else None,
                     specimen_id=self.specimen.text(), auto_save=self.auto.isChecked(),
                     min_interval_s=self.interval.value(), periodic_save=self.periodic.isChecked(),
                     periodic_interval_s=self.period.value())
            if request is not None:
                c['manual_request'] = request
            atomic_json(self.root / 'controls.json', controls(c))
            self.set_distance_value(c['distance_m'])
            self.info.setText('采样条件已保存；将在后续保存的图片中记录。')
            return True
        except (ValueError, OSError) as error:
            self.info.setText('设置未保存：' + str(error))
            return False

    def request_frame(self):
        request = {'id': uuid.uuid4().hex, 'time': time.time()}
        if self.apply_controls(request):
            self.manual_id = request['id']
            self.manual_at = request['time']
            self.manual.setEnabled(False)
            self.info.setText('已请求保存当前新鲜匹配帧；没有新鲜帧时等待下一帧，10秒内有效。')

    def refresh(self):
        status = read_json(self.root / 'status.json', {})
        live = bool(status.get('running') and time.time() - status.get('updated_at', 0) < 3)
        fresh = live and time.time() - (status.get('last_matched_at') or 0) < 2
        pending = self.manual_id is not None and time.time() - self.manual_at <= 10
        if self.manual_id and status.get('last_manual_saved') == self.manual_id:
            self.info.setText('手动保存完成，原图、带框图和检测信息已写入磁盘。')
            self.manual_id = None
            pending = False
        elif self.manual_id and not pending:
            self.info.setText('手动保存尚未确认成功；请检查采集状态和磁盘错误后重试。')
            self.manual_id = None
        self.manual.setEnabled(fresh and not pending)
        prefix = '在线保存中' if fresh else '等待原图与检测框配对' if live else '采集节点未运行；可以查看历史'
        text = '{} | 本次累计保存 {} 组 | 等待写盘 {} | 队列丢弃 {} | 写入失败 {}'.format(
            prefix, status.get('saved', 0), status.get('queued', 0),
            status.get('queue_dropped', 0), status.get('write_errors', 0))
        error = status.get('last_error') or status.get('config_error')
        self.status.setText(text + (' | ' + error if error else ''))
        catalog = read_json(self.root / 'catalog.json', {'events': []})
        recent = catalog.get('events', []) if isinstance(catalog, dict) else []
        recent = [entry for entry in recent if isinstance(entry, dict) and isinstance(entry.get('path'), str)]
        signature = tuple((entry['path'], entry.get('created_at')) for entry in recent)
        missing_current = False
        if self.current_path:
            missing_current = not self.capture_exists(self.current_path)
        if signature == self.catalog_signature and not missing_current:
            self.update_navigation()
            return
        self.catalog_signature = signature
        for entry in recent:
            try:
                if (capture_folder(self.root, entry['path']) / 'capture.json').is_file():
                    self.known_entries[entry['path']] = entry
                else:
                    self.known_entries.pop(entry['path'], None)
            except (ValueError, OSError):
                continue
        if missing_current:
            self.known_entries.pop(self.current_path, None)
            self.clear_current()
        self.known_entries = {path: entry for path, entry in self.known_entries.items()
                              if self.capture_exists(path)}
        self.rebuild_events()

    def history_loaded(self, events):
        for entry in events:
            try:
                if (capture_folder(self.root, entry['path']) / 'capture.json').is_file():
                    self.known_entries.setdefault(entry['path'], entry)
            except (OSError, ValueError):
                continue
        self.history_ready = True
        self.rebuild_events()

    def history_failed(self, error):
        self.history_ready = True
        self.update_navigation()
        self.info.setText('部分历史图片未能载入：' + error)

    def rebuild_events(self, preferred=None):
        entries = self.filtered_entries(sorted(self.known_entries.values(), key=event_order))
        signature = tuple((entry['path'], entry.get('created_at')) for entry in entries)
        if signature == self.events_signature and preferred is None:
            self.update_navigation()
            return
        self.events_signature = signature
        self.entries = entries
        old = self.current_path
        self.events.blockSignals(True)
        self.events.clear()
        for event in entries:
            self.events.addItem('{} | {}框 | {}m / {}'.format(
                saved_name(event), event.get('boxes', 0),
                event.get('distance_m') if event.get('distance_m') is not None else '?',
                {'bright': '明', 'dark': '暗'}.get(event.get('light'), '?')), event['path'])
        if preferred is not None:
            choice = self.events.findData(preferred)
        else:
            choice = len(entries) - 1 if self.follow.isChecked() or old is None else self.events.findData(old)
        self.events.setCurrentIndex(choice)
        self.events.blockSignals(False)
        if choice >= 0 and self.events.itemData(choice) != old:
            self.load_event(self.events.itemData(choice))
        elif not entries:
            self.clear_current()
        self.update_navigation()

    def filtered_entries(self, entries):
        return entries

    def update_navigation(self):
        index, count = self.events.currentIndex(), self.events.count()
        can_navigate = count > 1 and index >= 0
        self.previous_button.setEnabled(can_navigate)
        self.next_button.setEnabled(can_navigate)
        self.view.set_photo_navigation(can_navigate)
        loaded = bool(self.current_path and self.metadata and self.loaded_hash)
        self.delete_button.setEnabled(loaded)
        self.rename_button.setEnabled(loaded)
        self.position.setText('第 {} / {} 组{}'.format(index + 1 if index >= 0 else 0, count,
                              '' if self.history_ready else ' · 读取历史中'))

    def capture_exists(self, relative):
        try:
            return (capture_folder(self.root, relative) / 'capture.json').is_file()
        except (ValueError, OSError):
            return False

    def navigate(self, step):
        index, count = self.events.currentIndex(), self.events.count()
        if count > 0 and index >= 0:
            target = (index + step) % count
            self.follow.setChecked(False)
            self.events.setCurrentIndex(target)

    def clear_current(self):
        self.current_path = self.metadata = self.loaded_hash = None
        self.view.scene().clear()
        self.view.size, self.view.points, self.view.boxes = (0, 0), [], []
        self.view.sections = []
        self.view.setSceneRect(0, 0, 0, 0)
        self.view.viewport().update()
        self.box.blockSignals(True)
        self.box.clear()
        self.box.blockSignals(False)
        self.open_group.setEnabled(False)
        self.delete_button.setEnabled(False)
        self.rename_button.setEnabled(False)
        self.show_measurement()

    def prompt_rename(self, current_name):
        dialog = QtWidgets.QInputDialog(self)
        dialog.setWindowTitle('重命名整组照片')
        dialog.setInputMode(QtWidgets.QInputDialog.TextInput)
        dialog.setLabelText('输入本组新名称（例如：桥墩A_1.2m_明_清晰）：\n'
                            '整组文件夹及配套图片一起改名，已有像素测量保留。')
        dialog.setTextValue(current_name)
        dialog.setOkButtonText('重命名')
        dialog.setCancelButtonText('取消')
        dialog.resize(560, 180)
        accepted = dialog.exec_() == QtWidgets.QDialog.Accepted
        return dialog.textValue(), accepted

    def rename_current(self):
        if not self.current_path or not self.metadata:
            return
        self.follow.setChecked(False)
        # Capture identity before opening the modal; incoming frames may still
        # refresh the list while the user is editing the name.
        relative, original_hash, metadata = self.current_path, self.loaded_hash, dict(self.metadata)
        selected_box = self.box.currentData()
        transform, center = self.view.transform(), self.view.mapToScene(self.view.viewport().rect().center())
        new_name, accepted = self.prompt_rename(Path(relative).name)
        if not accepted:
            return
        try:
            result = rename_capture(self.root, relative, new_name, original_hash, metadata.get('created_at'))
        except (OSError, ValueError) as error:
            self.info.setText('重命名未完成：' + str(error))
            return
        if not result['changed']:
            self.info.setText('名称未改变。')
            return
        self.known_entries.pop(relative, None)
        self.known_entries[result['path']] = entry_from_metadata(result['path'], result['metadata'])
        self.events_signature = None
        self.rebuild_events(preferred=result['path'])
        box_index = self.box.findData(selected_box)
        if box_index >= 0:
            self.box.setCurrentIndex(box_index)
        self.view.setTransform(transform)
        self.view.centerOn(center)
        self.info.setText('本组已重命名为“{}”；原图、检测图、全部裁剪和测量记录均在新文件夹内。'.format(new_name))

    def delete_from_key(self):
        # Delete keeps its normal text-editing meaning in entry fields and
        # dialogs. Holding the key does not repeatedly remove following groups.
        if (QtWidgets.QApplication.activeModalWidget() is not None or
                QtWidgets.QApplication.activePopupWidget() is not None):
            return
        focus = QtWidgets.QApplication.focusWidget()
        while focus is not None and focus is not self:
            if isinstance(focus, (QtWidgets.QLineEdit, QtWidgets.QTextEdit,
                                  QtWidgets.QPlainTextEdit, QtWidgets.QAbstractSpinBox)):
                return
            if isinstance(focus, QtWidgets.QComboBox) and focus.isEditable():
                return
            focus = focus.parentWidget()
        if self.delete_button.isEnabled():
            self.delete_current()

    def remove_from_view(self, relative):
        index = self.events.findData(relative)
        remaining = [entry for entry in self.entries if entry['path'] != relative]
        preferred = remaining[min(max(index, 0), len(remaining) - 1)]['path'] if remaining else None
        self.known_entries.pop(relative, None)
        self.clear_current()
        self.events_signature = None
        self.rebuild_events(preferred)

    def delete_current(self):
        if not self.current_path or not self.metadata:
            return
        self.follow.setChecked(False)
        # Freeze the displayed identity: a concurrent new save must not redirect
        # this action to another group while the shared catalog lock is acquired.
        relative, original_hash, metadata = self.current_path, self.loaded_hash, dict(self.metadata)
        try:
            result = delete_capture(self.root, relative, original_hash, metadata.get('created_at'))
        except (OSError, ValueError) as error:
            if not self.capture_exists(relative):
                self.remove_from_view(relative)
            self.info.setText('删除未完成：' + str(error))
            return
        self.remove_from_view(relative)
        self.info.setText('已删除整组“{}”（{}个文件）；其他图片组保留。'.format(
            saved_name(metadata), result['files']))

    def stop_history(self):
        if self.history_loader.isRunning():
            self.history_loader.requestInterruption()
            self.history_loader.wait(3000)

    def closeEvent(self, event):
        self.timer.stop()
        self.stop_history()
        super().closeEvent(event)

    def select_event(self, index):
        if index >= 0:
            self.follow.setChecked(False)
            self.load_event(self.events.itemData(index))

    def open_current_folder(self):
        if self.current_path:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(inside(self.root, self.current_path))))

    def follow_changed(self, enabled):
        if enabled and self.entries:
            index = len(self.entries) - 1
            self.events.blockSignals(True)
            self.events.setCurrentIndex(index)
            self.events.blockSignals(False)
            if self.events.itemData(index) != self.current_path:
                self.load_event(self.events.itemData(index))
        self.update_navigation()

    def load_event(self, relative):
        try:
            folder = capture_folder(self.root, relative)
            metadata = read_json(folder / 'capture.json')
            if not metadata:
                raise ValueError('图片组尚未完成或元数据无法读取')
            image_bytes = inside(folder, metadata['original']).read_bytes()
            actual_hash = hashlib.sha256(image_bytes).hexdigest()
            if actual_hash != metadata['original_sha256']:
                raise ValueError('原图内容与保存记录不一致，停止测量该图')
            image = QtGui.QImage.fromData(image_bytes)
            if image.isNull() or [image.width(), image.height()] != metadata['size']:
                raise ValueError('原图损坏或尺寸不一致')
            self.current_path, self.metadata, self.loaded_hash = relative, metadata, actual_hash
            self.known_entries.setdefault(relative, entry_from_metadata(relative, metadata))
            self.open_group.setEnabled(True)
            self.view.set_image(image, metadata['boxes'])
            self.box.blockSignals(True)
            self.box.clear()
            for b in metadata['boxes']:
                self.box.addItem('裂缝框 #{} · {:.1f}%'.format(b['id'] + 1, b['confidence'] * 100), b['id'])
            self.box.addItem('整张原图 / 检查漏检位置', 'full')
            selected = self.box.findData(self.preferred_box)
            if selected < 0:
                selected = 0  # first crack, or full image when no boxes exist
            self.box.setCurrentIndex(selected)
            self.box.blockSignals(False)
            self.select_box(selected, remember=False)
            c = metadata['conditions']
            self.info.setText('已保存帧 {} · {}×{} · 距离 {}m / {} · {}。'.format(
                saved_name(metadata), *metadata['size'],
                c['distance_m'] if c['distance_m'] is not None else '未设置',
                {'bright': '明', 'dark': '暗'}.get(c['light'], '未设置'),
                {'detection': '检测触发', 'manual': '手动保存', 'periodic': '定时采样'}[metadata['reason']]))
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.clear_current()
            self.info.setText('读取失败：' + str(error))
        self.update_navigation()

    def box_key(self):
        return str(self.box.currentData())

    def box_activated(self, _):
        self.follow.setChecked(False)
        # Reselecting the already displayed full-image fallback is also an
        # explicit preference, even though currentIndexChanged does not fire.
        if self.metadata and self.box.currentIndex() >= 0:
            self.preferred_box = self.box.currentData()

    def select_box(self, _, remember=True):
        if not self.metadata:
            return
        if remember and self.box.currentIndex() >= 0:
            self.preferred_box = self.box.currentData()
        self.view.points = []
        saved = read_json(inside(self.root, self.current_path) / 'measurements.json', {})
        measurement = saved.get('measurements', {}).get(self.box_key(), {})
        points = measurement.get('endpoints', [])
        if (saved.get('original_sha256') == self.loaded_hash and len(points) == 2 and
                all(isinstance(p, list) and len(p) == 2 and
                    all(type(n) in (float, int) and math.isfinite(n) for n in p) and
                    0 <= p[0] <= self.view.size[0] and 0 <= p[1] <= self.view.size[1]
                    for p in points)):
            self.view.points = points
        self.fit_selected(pause=False)
        self.show_measurement()

    def fit_selected(self, pause=True):
        if pause:
            self.follow.setChecked(False)
        if not self.metadata:
            return
        box = next((b for b in self.metadata['boxes'] if b['id'] == self.box.currentData()), None)
        self.view.fit(box['xyxy'] if box else None, self.context_margin.currentData())

    def overlay_changed(self, enabled):
        self.view.show_boxes = enabled
        self.view.viewport().update()

    def show_measurement(self):
        points = self.view.points
        text = '网格每格＝1原图像素；放大至8倍以上，沿垂直裂缝走向点两侧边界。'
        if len(points) == 1:
            text = '已暂停跟随。已选第一侧边界，请点另一侧。'
        elif len(points) == 2:
            width = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
            text = '手工截面宽度 {:.2f} px；原图坐标 {} → {}。测的是裂缝两侧间距，不是框宽。'.format(
                width, points[0], points[1])
        self.measurement.setText(text)

    def save_measurement(self, points):
        self.show_measurement()
        if not self.metadata or len(points) != 2:
            return
        self.persist_measurement(points)

    def persist_measurement(self, points):
        try:
            with catalog_lock(self.root):
                folder, _ = checked_metadata(self.root, self.current_path, self.loaded_hash,
                                              self.metadata.get('created_at'))
                path = folder / 'measurements.json'
                saved = read_json(path, {})
                if saved.get('original_sha256') != self.loaded_hash:
                    saved = {'schema_version': 1, 'original_sha256': self.loaded_hash,
                             'ros_stamp': self.metadata['ros_stamp'], 'conditions': self.metadata['conditions'],
                             'measurements': {}}
                if points:
                    saved['measurements'][self.box_key()] = {
                        'endpoints': points,
                        'width_px': math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1]),
                        'unit': 'original_image_px', 'method': 'manual_boundary_grid',
                        'updated_at': datetime.now(timezone.utc).isoformat()}
                else:
                    saved['measurements'].pop(self.box_key(), None)
                atomic_json(path, saved)
        except (OSError, ValueError) as error:
            self.info.setText('测量未能写入磁盘：' + str(error))

    def clear_measurement(self):
        self.follow.setChecked(False)
        self.view.points = []
        self.view.viewport().update()
        if self.metadata:
            self.persist_measurement(None)
        self.show_measurement()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--tiled', action='store_true', help='Open normally for the three-window experiment layout')
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    from crack_experiment_viewer import ExperimentWindow
    window = ExperimentWindow(args.root)
    if args.tiled:
        window.show()
    else:
        window.showMaximized()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
