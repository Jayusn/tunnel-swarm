"""Shared NUC/Windows experiment tools layered on the existing photo viewer."""
from copy import deepcopy

from PyQt5 import QtCore, QtGui, QtWidgets
from crack_pixel_viewer import Window, DISTANCE_PRESETS
from crack_capture_catalog import capture_folder, entry_from_metadata
from crack_live_common import read_json
from crack_measurements import (load_measurements, sections, change_measurement,
                                undo_measurement, experiment_tags, save_tags_and_rename, tagged_capture_name)
from crack_experiment_export import export_figure, export_csv, LIGHT_LABELS, REVIEW_LABELS


class ExperimentWindow(Window):
    def __init__(self, root):
        self.measurement_undo = {}
        self.tag_cache = {}
        self.current_sections = []
        self._loading_sections = False
        super().__init__(root)
        self.compact_layout()
        screen = QtWidgets.QApplication.primaryScreen()
        area = screen.availableGeometry() if screen else QtCore.QRect(0, 0, 1280, 1020)
        self.resize(min(1260, area.width()-30), min(960, area.height()-60))

    def build_ui(self):
        super().build_ui()
        layout = self.centralWidget().layout()
        # Keep acquisition conditions and Apply visible above the image.
        # Only infrequent save options and filters belong in the details panel.
        self.detail_rows = [layout.itemAt(2).layout()]
        self.capture_help = layout.itemAt(3).widget()
        for button in self.findChildren(QtWidgets.QPushButton):
            if button.text() == '清除截面':
                button.setText('删除所选截面')
                button.setToolTip('删除所选测量记录；可使用“撤销测量”恢复。')
        row = QtWidgets.QHBoxLayout()
        self.section_combo = QtWidgets.QComboBox()
        self.section_combo.setMinimumWidth(200)
        self.section_combo.currentIndexChanged.connect(self.section_selected)
        row.addWidget(self.section_combo, 1)
        self.replace_section = QtWidgets.QCheckBox('覆盖所选截面')
        self.replace_section.setToolTip('默认每次测量新增截面；勾选后重新点两侧边界才会覆盖当前所选记录。')
        row.addWidget(self.replace_section)
        for text, callback in [('新增截面', self.new_section), ('撤销测量', self.undo_last),
                               ('导出测量图', self.export_selected), ('导出当前列表 CSV', self.export_table)]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.insertLayout(layout.indexOf(self.view), row)
        row = QtWidgets.QHBoxLayout()
        self.tag_summary = QtWidgets.QLabel('实验标签：未选择照片')
        self.tag_summary.setWordWrap(True)
        row.addWidget(self.tag_summary, 1)
        edit = QtWidgets.QPushButton('补录本组实验标签')
        edit.clicked.connect(self.edit_tags)
        row.addWidget(edit)
        layout.insertLayout(layout.indexOf(self.view), row)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('筛选'))
        self.filter_distance = QtWidgets.QComboBox()
        for label, value in [('全部距离', 'all'), ('距离未设', None)] + [('{:g} m'.format(x), x) for x in DISTANCE_PRESETS]:
            self.filter_distance.addItem(label, value)
        self.filter_distance.setToolTip('列表自动包含已保存照片的自定义距离。')
        self.filter_light = QtWidgets.QComboBox()
        for label, value in [('全部光照', 'all')] + [(v, k) for k, v in LIGHT_LABELS.items()]:
            self.filter_light.addItem(label, value)
        self.filter_measure = QtWidgets.QComboBox()
        for label, value in [('全部测量状态', 'all'), ('已测量', True), ('未测量', False)]:
            self.filter_measure.addItem(label, value)
        self.filter_review = QtWidgets.QComboBox()
        self.filter_review.addItem('全部复核状态', 'all')
        for value, label in REVIEW_LABELS.items():
            self.filter_review.addItem(label, value)
        for widget in (self.filter_distance, self.filter_light, self.filter_measure, self.filter_review):
            row.addWidget(widget)
            widget.currentIndexChanged.connect(self.filters_changed)
        reset = QtWidgets.QPushButton('清除筛选')
        reset.clicked.connect(self.reset_filters)
        row.addWidget(reset)
        self.detail_rows.append(row)
        layout.insertLayout(layout.indexOf(self.view), row)
        self.measurement.setToolTip('整数原图坐标之间的欧氏距离；小数不是亚像素测量精度。不同照片的A/B/C位置需人工对应。')

    def compact_layout(self):
        layout = self.centralWidget().layout()
        self.toolbar_area = QtWidgets.QWidget()
        toolbar = QtWidgets.QVBoxLayout(self.toolbar_area)
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.setSpacing(3)
        self.detail_panel = QtWidgets.QWidget()
        details = QtWidgets.QVBoxLayout(self.detail_panel)
        details.setContentsMargins(0, 0, 0, 0)
        details.setSpacing(3)
        bar = QtWidgets.QHBoxLayout()
        self.image_status = QtWidgets.QLabel('图片优先显示')
        self.image_status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        bar.addWidget(self.image_status, 1)
        self.detail_toggle = QtWidgets.QPushButton('更多设置 / 筛选')
        self.detail_toggle.setCheckable(True)
        self.detail_toggle.toggled.connect(self.toggle_details)
        bar.addWidget(self.detail_toggle)
        focus = QtWidgets.QPushButton('专注看图（F11）')
        focus.clicked.connect(self.toggle_focus)
        bar.addWidget(focus)
        toolbar.addLayout(bar)
        detail_rows = self.detail_rows + ([self.sync_row] if hasattr(self, 'sync_row') else [])
        while layout.itemAt(0).widget() is not self.view:
            item = layout.takeAt(0)
            if item.layout():
                row = item.layout()
                target = details if row in detail_rows else toolbar
                row.setParent(None)
                target.addLayout(row)
            else:
                widget = item.widget()
                target = details if widget in (self.status, self.capture_help) else toolbar
                target.addWidget(widget)
        toolbar.addWidget(self.detail_panel)
        self.detail_panel.hide()
        layout.insertWidget(0, self.toolbar_area)
        layout.setContentsMargins(5, 4, 5, 4)
        layout.setSpacing(3)
        self.focus_hint = QtWidgets.QLabel('专注看图：F11 或 Esc 返回；← / → 翻页，滚轮缩放，右键拖动。')
        layout.addWidget(self.focus_hint)
        self.focus_hint.hide()
        self.focus_mode = False
        self.focus_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence('F11'), self)
        self.focus_shortcut.activated.connect(self.toggle_focus)
        self.escape_focus = QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), self)
        self.escape_focus.activated.connect(lambda: self.toggle_focus() if self.focus_mode else None)
        self.focus_navigation = []
        for key, step in [('Left', -1), ('Right', 1)]:
            shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
            shortcut.activated.connect(lambda s=step: self.navigate(s))
            shortcut.setEnabled(False)
            self.focus_navigation.append(shortcut)
        self.update_compact_status()

    def toggle_details(self, expanded):
        self.detail_panel.setVisible(expanded)
        self.detail_toggle.setText('收起更多设置' if expanded else '更多设置 / 筛选')

    def toggle_focus(self):
        if QtWidgets.QApplication.activeModalWidget() is not None:
            return
        self.focus_mode = not self.focus_mode
        if self.focus_mode:
            self.was_maximized = self.isMaximized()
        self.toolbar_area.setVisible(not self.focus_mode)
        self.info.setVisible(not self.focus_mode)
        self.focus_hint.setVisible(self.focus_mode)
        for shortcut in self.focus_navigation:
            shortcut.setEnabled(self.focus_mode)
        if self.focus_mode:
            self.showFullScreen()
        elif self.was_maximized:
            self.showMaximized()
        else:
            self.showNormal()

    def update_compact_status(self):
        if not hasattr(self, 'image_status'):
            return
        # Keep detailed status readable on hover without spending image height.
        text = self.status.text().split('\n')[0]
        short = text if len(text) <= 85 else text[:82] + '…'
        active = sum(w.currentIndex() != 0 for w in
                     (self.filter_distance, self.filter_light, self.filter_measure, self.filter_review))
        self.image_status.setText(short + (' | 筛选已启用' if active else ''))
        self.image_status.setToolTip(self.status.text())

    def refresh(self):
        super().refresh()
        self.update_compact_status()

    def select_box(self, index, remember=True):
        super().select_box(index, remember)
        self.replace_section.setChecked(False)
        self.load_sections()
        self.update_tag_summary()

    def clear_current(self):
        super().clear_current()
        if hasattr(self, 'section_combo'):
            self.current_sections = []
            self.section_combo.clear()
            self.tag_summary.setText('实验标签：未选择照片')

    def load_sections(self, preferred=None):
        self.current_sections = []
        if self.metadata:
            try:
                saved = load_measurements(capture_folder(self.root, self.current_path), self.metadata)
                self.current_sections = sections(saved, self.box_key())
            except (ValueError, OSError, KeyError, TypeError) as error:
                self.info.setText('读取测量失败：' + str(error))
        self._loading_sections = True
        self.section_combo.clear()
        for section in self.current_sections:
            self.section_combo.addItem('截面 {} · {:.2f} px'.format(section['label'], section['width_px']), section['id'])
        if self.current_sections:
            index = self.section_combo.findData(preferred)
            self.section_combo.setCurrentIndex(index if index >= 0 else len(self.current_sections)-1)
        else:
            self.section_combo.addItem('尚无截面：放大后点击两侧边界', None)
        self._loading_sections = False
        self.view.sections = deepcopy(self.current_sections)
        self.section_selected(self.section_combo.currentIndex(), pause=False)

    def selected_section(self):
        identity = self.section_combo.currentData()
        return next((s for s in self.current_sections if s['id'] == identity), None)

    def section_selected(self, _, pause=True):
        if self._loading_sections:
            return
        if pause:
            self.follow.setChecked(False)
        section = self.selected_section()
        self.view.points = deepcopy(section['endpoints']) if section else []
        self.view.viewport().update()
        self.show_measurement()

    def new_section(self):
        self.follow.setChecked(False)
        self.replace_section.setChecked(False)
        self.view.points = []
        self.view.viewport().update()
        self.show_measurement()
        self.info.setText('放大至8倍以上，在新的截面两侧各点一下；第二点后自动新增并保存。')

    def save_measurement(self, points):
        self.show_measurement()
        if not self.metadata or len(points) != 2:
            return
        section = self.selected_section()
        if self.replace_section.isChecked() and section is None:
            self.info.setText('尚未选择截面；取消覆盖勾选可新增。')
            return
        identity = section['id'] if self.replace_section.isChecked() else None
        try:
            before, after, changed = change_measurement(self.root, self.current_path, self.metadata,
                                                        self.box_key(), points, identity)
            self.remember_undo(before, after)
            self.load_sections(changed['id'])
            self.refresh_filters_after_edit()
            self.info.setText('已自动保存截面 {}：{:.2f} px。{}；记录位于本组 measurements.json。'.format(
                changed['label'], changed['width_px'], '当前为覆盖模式' if identity else '下次测量继续新增截面'))
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.info.setText('测量未保存：' + str(error))

    def remember_undo(self, before, after):
        identity = (self.metadata['original_sha256'], self.metadata.get('created_at'))
        history = self.measurement_undo.setdefault(identity, [])
        history.append((before, after))
        del history[:-30]

    def undo_last(self):
        if not self.metadata:
            return
        history = self.measurement_undo.get((self.loaded_hash, self.metadata.get('created_at')), [])
        if not history:
            self.info.setText('本组当前会话没有可撤销的测量操作。')
            return
        try:
            before, after = history[-1]
            undo_measurement(self.root, self.current_path, self.metadata, before, after)
            history.pop()
            self.view.points = []
            self.load_sections()
            self.refresh_filters_after_edit()
            self.info.setText('已撤销本组上一次测量新增、覆盖或删除，并保存。')
        except (OSError, ValueError) as error:
            self.info.setText('无法撤销：' + str(error))

    def clear_measurement(self):
        self.follow.setChecked(False)
        # A single pending click can be cleared without deleting a saved section.
        if len(self.view.points) == 1:
            self.view.points = []
            self.view.viewport().update()
            self.show_measurement()
            self.info.setText('已取消尚未完成的测量。')
            return
        section = self.selected_section()
        if not self.metadata or not section:
            return
        try:
            before, after, removed = change_measurement(self.root, self.current_path, self.metadata,
                self.box_key(), section_id=section['id'], delete=True)
            self.remember_undo(before, after)
            self.load_sections()
            self.refresh_filters_after_edit()
            self.info.setText('已删除截面 {}；可点“撤销测量”恢复。'.format(removed['label']))
        except (OSError, ValueError) as error:
            self.info.setText('删除截面失败：' + str(error))

    def export_selected(self):
        section = self.selected_section()
        if not self.metadata or not section:
            self.info.setText('请先完成测量并选择一个截面。')
            return
        try:
            path = export_figure(self.root, self.current_path, self.metadata, self.box_key(), section)
            self.info.setText('测量图已保存：' + str(path) + '（点击“打开本组”查看）')
        except (OSError, ValueError, KeyError) as error:
            self.info.setText('导出失败：' + str(error))

    def export_table(self):
        if not self.history_ready:
            self.info.setText('历史列表仍在载入，请稍后导出。')
            return
        if not self.entries:
            self.info.setText('当前筛选列表为空，没有可导出的照片组。')
            return
        try:
            path, count = export_csv(self.root, list(self.entries))
            self.info.setText('已导出当前筛选列表：{} 行 → {}。未测量和无框照片也会保留。'.format(count, path))
        except (OSError, ValueError, KeyError) as error:
            self.info.setText('导出失败：' + str(error))

    def tags_and_measured(self, relative, meta=None):
        folder = capture_folder(self.root, relative)
        paths = [folder / n for n in ('capture.json', 'experiment.json', 'measurements.json')]
        signature = tuple((p.stat().st_mtime_ns, p.stat().st_size) if p.exists() else None for p in paths)
        cached = self.tag_cache.get(relative)
        if cached and cached[0] == signature:
            return cached[1], cached[2]
        meta = meta or read_json(paths[0])
        tags = experiment_tags(folder, meta)
        saved = load_measurements(folder, meta)
        measured = any(v.get('sections') for v in saved['measurements'].values())
        self.tag_cache[relative] = (signature, tags, measured)
        return tags, measured

    def filtered_entries(self, entries):
        # Discover custom distances before applying any filters, preserving selection.
        distances = set(DISTANCE_PRESETS)
        tagged = []
        for entry in entries:
            try:
                tags, measured = self.tags_and_measured(entry['path'])
                distance = tags.get('distance_m')
                if type(distance) in (int, float) and 0 < distance <= 1000:
                    distances.add(distance)
                tagged.append((entry, tags, measured))
            except (OSError, ValueError, KeyError, TypeError):
                tagged.append((entry, None, False))
        selected = self.filter_distance.currentData()
        if type(selected) in (int, float):
            distances.add(selected)
        values = ['all', None] + sorted(distances)
        if [self.filter_distance.itemData(i) for i in range(self.filter_distance.count())] != values:
            self.filter_distance.blockSignals(True)
            try:
                self.filter_distance.clear()
                self.filter_distance.addItem('全部距离', 'all')
                self.filter_distance.addItem('距离未设', None)
                for distance in sorted(distances):
                    self.filter_distance.addItem('{:g} m'.format(distance), distance)
                self.filter_distance.setCurrentIndex(max(0, self.filter_distance.findData(selected)))
            finally:
                self.filter_distance.blockSignals(False)
        result = []
        for entry, tags, measured in tagged:
            try:
                if tags is None:
                    raise ValueError('Invalid capture metadata')
                if self.filter_distance.currentData() != 'all' and tags.get('distance_m') != self.filter_distance.currentData():
                    continue
                if self.filter_light.currentData() != 'all' and tags.get('light', 'unknown') != self.filter_light.currentData():
                    continue
                if self.filter_measure.currentData() != 'all' and measured != self.filter_measure.currentData():
                    continue
                if self.filter_review.currentData() != 'all' and tags.get('review') != self.filter_review.currentData():
                    continue
                result.append(dict(entry, distance_m=tags.get('distance_m'), light=tags.get('light')))
            except (OSError, ValueError, KeyError, TypeError):
                # Keep damaged metadata visible in an unfiltered list for diagnosis.
                if all(w.currentIndex() == 0 for w in (self.filter_distance, self.filter_light, self.filter_measure, self.filter_review)):
                    result.append(entry)
        return result

    def filters_changed(self):
        self.follow.setChecked(False)
        self.events_signature = None
        self.rebuild_events()
        if self.events.currentIndex() < 0 and self.entries:
            self.events.setCurrentIndex(0)
        self.update_compact_status()

    def reset_filters(self):
        for widget in (self.filter_distance, self.filter_light, self.filter_measure, self.filter_review):
            widget.blockSignals(True)
            widget.setCurrentIndex(0)
            widget.blockSignals(False)
        self.filters_changed()

    def refresh_filters_after_edit(self):
        self.tag_cache.pop(self.current_path, None)
        self.events_signature = None
        self.rebuild_events()
        if self.events.currentIndex() < 0 and self.entries:
            self.events.setCurrentIndex(0)

    def update_tag_summary(self):
        if not self.metadata:
            self.tag_summary.setText('实验标签：未选择照片')
            return
        tags = experiment_tags(capture_folder(self.root, self.current_path), self.metadata)
        self.tag_summary.setText('实验标签：{} m / {} / 编号 {} / {}'.format(
            tags.get('distance_m') or '未设置', LIGHT_LABELS.get(tags.get('light'), '未设置'),
            tags.get('specimen_id') or '未设置', REVIEW_LABELS.get(tags.get('review'), '未复核')))

    def edit_tags(self):
        if not self.metadata:
            return
        self.follow.setChecked(False)
        relative, meta = self.current_path, deepcopy(self.metadata)
        selected_box = self.box.currentData()
        selected_section = self.section_combo.currentData()
        transform, center = self.view.transform(), self.view.mapToScene(self.view.viewport().rect().center())
        saved_result = {}
        tags = experiment_tags(capture_folder(self.root, relative), meta)
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('本组实验标签（仅当前保存副本）')
        form = QtWidgets.QFormLayout(dialog)
        distance = QtWidgets.QDoubleSpinBox()
        distance.setRange(0, 1000)
        distance.setDecimals(3)
        distance.setSingleStep(0.1)
        distance.setSpecialValueText('未设置')
        distance.setSuffix(' m')
        distance.setValue(tags.get('distance_m') or 0)
        light = QtWidgets.QComboBox()
        for value, label in LIGHT_LABELS.items():
            light.addItem(label, value)
        light.setCurrentIndex(max(0, light.findData(tags.get('light'))))
        specimen = QtWidgets.QLineEdit(tags.get('specimen_id', ''))
        lux = QtWidgets.QLineEdit('' if tags.get('lux') is None else str(tags['lux']))
        notes = QtWidgets.QLineEdit(tags.get('notes', ''))
        review = QtWidgets.QComboBox()
        for value, label in REVIEW_LABELS.items():
            review.addItem(label, value)
        review.setCurrentIndex(max(0, review.findData(tags.get('review'))))
        for label, widget in [('距离', distance), ('光照', light), ('裂缝编号', specimen),
                               ('照度 lux（可空）', lux), ('人工复核', review), ('备注', notes)]:
            form.addRow(label, widget)
        name_preview = QtWidgets.QLabel()
        name_preview.setWordWrap(True)
        form.addRow('保存后名称', name_preview)
        def preview():
            proposed, _ = tagged_capture_name(capture_folder(self.root, relative), meta,
                dict(tags, distance_m=distance.value() or None, light=light.currentData(), specimen_id=specimen.text()))
            name_preview.setText(proposed)
        distance.valueChanged.connect(preview)
        light.currentIndexChanged.connect(preview)
        specimen.textChanged.connect(preview)
        preview()
        error_label = QtWidgets.QLabel('保存时同步修改本组文件夹和配套图片名称；重名自动加序号，不覆盖其他组。')
        error_label.setWordWrap(True)
        form.addRow(error_label)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        form.addRow(buttons)
        def accept():
            try:
                updated = dict(tags, distance_m=distance.value() or None, light=light.currentData(),
                               specimen_id=specimen.text(), lux=float(lux.text()) if lux.text().strip() else None,
                               review=review.currentData(), notes=notes.text())
                saved_result.update(save_tags_and_rename(self.root, relative, meta, updated))
                dialog.accept()
            except (ValueError, OSError) as error:
                error_label.setText('标签未保存：' + str(error))
        buttons.accepted.connect(accept)
        buttons.rejected.connect(dialog.reject)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self.tag_cache.pop(relative, None)
            new_relative = saved_result['path']
            self.tag_cache.pop(new_relative, None)
            self.known_entries.pop(relative, None)
            self.known_entries[new_relative] = entry_from_metadata(new_relative, saved_result['metadata'])
            self.clear_current()
            self.events_signature = None
            self.rebuild_events(preferred=new_relative)
            if self.events.currentIndex() < 0 and self.entries:
                self.events.setCurrentIndex(0)
            if self.current_path == new_relative:
                box_index = self.box.findData(selected_box)
                if box_index >= 0:
                    self.box.setCurrentIndex(box_index)
                self.load_sections(selected_section)
                self.view.setTransform(transform)
                self.view.centerOn(center)
            self.update_tag_summary()
            self.info.setText('标签及整组名称已同步保存：{}。原图像素和测量记录保留。{}'.format(
                new_relative.split('/')[-1], '该组不符合当前筛选，已从列表隐藏。' if self.current_path != new_relative else ''))
