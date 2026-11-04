"""Export reproducible pixel inspection figures and UTF-8 CSV experiment rows."""
import csv
from datetime import datetime
import hashlib
import math
import uuid

from PyQt5 import QtCore, QtGui
from crack_capture_catalog import capture_folder, catalog_lock, checked_metadata
from crack_live_common import read_json
from crack_measurements import experiment_tags, load_measurements, sections

REVIEW_LABELS = {'unknown': '未复核', 'hit': '裂缝检出', 'miss': '存在漏检',
                 'false_positive': '存在误检', 'negative': '无裂缝对照', 'uncertain': '无法判断'}
LIGHT_LABELS = {'unknown': '未设置', 'bright': '明', 'dark': '暗'}


def export_figure(root, relative, meta, box_key, section):
    with catalog_lock(root):
        folder, actual = checked_metadata(root, relative, meta['original_sha256'], meta.get('created_at'))
        saved = load_measurements(folder, actual)
        section = next((s for s in sections(saved, box_key) if s['id'] == section['id']), None)
        if section is None:
            raise ValueError('截面已不存在，请重新选择')
        original = folder / actual['original']
        raw = original.read_bytes()
        if hashlib.sha256(raw).hexdigest() != actual['original_sha256']:
            raise ValueError('原图校验失败')
        source = QtGui.QImage.fromData(raw)
        if source.isNull():
            raise ValueError('原图无法读取')
        points = section['endpoints']
        left = max(0, math.floor(min(p[0] for p in points)) - 18)
        top = max(0, math.floor(min(p[1] for p in points)) - 18)
        right = min(source.width(), math.ceil(max(p[0] for p in points)) + 18)
        bottom = min(source.height(), math.ceil(max(p[1] for p in points)) + 18)
        width, height = right - left, bottom - top
        scale = max(1, min(16, 1800 // max(width, height)))
        header, footer = 145, 40
        canvas = QtGui.QImage(max(1000, width * scale), header + height * scale + footer,
                              QtGui.QImage.Format_RGB32)
        canvas.fill(QtGui.QColor('#182330'))
        painter = QtGui.QPainter(canvas)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        painter.setFont(QtGui.QFont('Microsoft YaHei' if __import__('os').name == 'nt' else 'Sans', 12))
        painter.setPen(QtGui.QColor('white'))
        tags = experiment_tags(folder, actual)
        box = next((b for b in actual['boxes'] if str(b['id']) == str(box_key)), None)
        confidence = '{:.1f}%'.format(box['confidence'] * 100) if box else '无指定检测框'
        lines = [folder.name,
                 '截面 {} | 手工宽度 {:.2f} px | 检测置信度 {}'.format(section['label'], section['width_px'], confidence),
                 '距离 {} m | 光照 {} | 裂缝编号 {}'.format(tags.get('distance_m') or '未设置',
                     LIGHT_LABELS.get(tags.get('light'), '未设置'), tags.get('specimen_id', '')),
                 '原图坐标 {} → {} | 显示放大 {} 倍；宽度单位为原图像素'.format(*points, scale),
                 '测量时间：' + section.get('updated_at', '')]
        metrics = painter.fontMetrics()
        for index, text in enumerate(lines):
            painter.drawText(12, 24 + 25 * index, metrics.elidedText(text, QtCore.Qt.ElideRight, canvas.width()-24))
        painter.drawImage(QtCore.QRect(0, header, width * scale, height * scale),
                          source, QtCore.QRect(left, top, width, height))
        if scale >= 8:
            painter.setPen(QtGui.QPen(QtGui.QColor(140, 200, 255, 110), 1))
            for x in range(width + 1):
                painter.drawLine(x * scale, header, x * scale, header + height * scale)
            for y in range(height + 1):
                painter.drawLine(0, header + y * scale, width * scale, header + y * scale)
        converted = [QtCore.QPointF((p[0]-left)*scale, header+(p[1]-top)*scale) for p in points]
        painter.setPen(QtGui.QPen(QtGui.QColor('#00ffff'), 3))
        painter.drawLine(*converted)
        for point in converted:
            painter.drawEllipse(point, 4, 4)
        painter.setPen(QtGui.QColor('white'))
        painter.drawText(12, canvas.height()-14, '每格＝1原图像素；人工选取两侧边界，非自动分割或亚像素测量。')
        painter.end()
        name = '测量_{}_{}_{}_{}.png'.format('全图' if box is None else '框'+str(box['id']+1),
                    section['label'], datetime.now().strftime('%H%M%S'), uuid.uuid4().hex[:6])
        path = folder / name
        temporary = folder / ('.export_' + uuid.uuid4().hex + '.png')
        try:
            if not canvas.save(str(temporary), 'PNG'):
                raise OSError('测量图写入失败')
            temporary.replace(path)
        finally:
            if temporary.exists():
                temporary.unlink()
        return path


def export_csv(root, entries):
    headers = ['照片组', '拍摄时间', '裂缝编号', '距离_m', '光照', '照度_lux', '人工复核',
               '备注', '保存原因', '检测框数', '框编号', '置信度', '截面', '宽度_px',
               'x1', 'y1', 'x2', 'y2', '测量时间', '原图宽', '原图高', '原图SHA256']
    rows = []
    # Snapshot under the same lock as rename/delete/measurement; never export a mixed group.
    with catalog_lock(root):
        for entry in entries:
            folder = capture_folder(root, entry['path'])
            meta = read_json(folder / 'capture.json')
            if not meta:
                raise ValueError('照片组已变化，请刷新后重新导出：' + entry['path'])
            tags = experiment_tags(folder, meta)
            saved = load_measurements(folder, meta)
            keys = [str(b['id']) for b in meta['boxes']]
            if sections(saved, 'full') or not keys:
                keys.append('full')
            for key in keys:
                box = next((b for b in meta['boxes'] if str(b['id']) == key), None)
                for section in sections(saved, key) or [None]:
                    coords = sum(section['endpoints'], []) if section else ['', '', '', '']
                    rows.append([entry['path'], meta.get('captured_at', meta.get('created_at')),
                        tags.get('specimen_id'), tags.get('distance_m'), LIGHT_LABELS.get(tags.get('light'), '未设置'),
                        tags.get('lux'), REVIEW_LABELS.get(tags.get('review'), '未复核'), tags.get('notes'),
                        meta.get('reason'), len(meta['boxes']), box['id']+1 if box else '全图',
                        box['confidence'] if box else '', section['label'] if section else '',
                        section['width_px'] if section else '', *coords,
                        section.get('updated_at') if section else '', *meta['size'], meta['original_sha256']])
    directory = root / 'exports'
    directory.mkdir(exist_ok=True)
    path = directory / ('裂缝实验_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6] + '.csv')
    temporary = path.with_suffix('.tmp')
    try:
        with temporary.open('w', encoding='utf-8-sig', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(headers)
            # User-entered labels must be text when opened in spreadsheet applications.
            writer.writerows([("'"+v if isinstance(v, str) and v.lstrip().startswith(('=', '+', '-', '@')) else v)
                               for v in row] for row in rows)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path, len(rows)
