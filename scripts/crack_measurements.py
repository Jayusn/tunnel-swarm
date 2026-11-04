"""Versioned manual cross sections and local experiment annotations (no Qt/ROS)."""
from copy import deepcopy
from datetime import datetime, timezone
import math
import re
import uuid

from crack_live_common import read_json, atomic_json
from crack_capture_catalog import catalog_lock, checked_metadata, capture_folder, capture_name, rename_capture


def section_label(index):
    value = ''
    while index >= 0:
        value = chr(65 + index % 26) + value
        index = index // 26 - 1
    return value


def load_measurements(folder, meta):
    path = folder / 'measurements.json'
    saved = read_json(path)
    if path.exists() and not isinstance(saved, dict):
        raise ValueError('测量文件无法读取，保留原文件，请检查后重试')
    if saved and saved.get('original_sha256') != meta['original_sha256']:
        raise ValueError('测量记录与当前原图不一致')
    saved = deepcopy(saved or {'original_sha256': meta['original_sha256'],
                              'ros_stamp': meta.get('ros_stamp'),
                              'conditions': meta.get('conditions', {}), 'measurements': {}})
    if not isinstance(saved.get('measurements'), dict):
        raise ValueError('测量记录格式无效')
    for key, value in saved['measurements'].items():
        if 'sections' not in value:
            points = value.get('endpoints', [])
            value['sections'] = [dict(value, id='legacy_' + key, label='A')] if len(points) == 2 else []
        for section in value['sections']:
            points = section.get('endpoints', [])
            if (len(points) != 2 or any(len(p) != 2 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in p) for p in points)):
                raise ValueError('测量坐标无效')
    saved['schema_version'] = 2
    return saved


def sections(saved, box_key):
    return saved.get('measurements', {}).get(str(box_key), {}).get('sections', [])


def rebuild_compatibility(value):
    # Older viewers can read the latest cross section; all sections remain in v2.
    for key in ('endpoints', 'width_px', 'unit', 'method', 'updated_at'):
        value.pop(key, None)
    if value['sections']:
        value.update({key: value['sections'][-1][key] for key in
                      ('endpoints', 'width_px', 'unit', 'method', 'updated_at')
                      if key in value['sections'][-1]})


def change_measurement(root, relative, meta, box_key, points=None, section_id=None, delete=False):
    with catalog_lock(root):
        folder, actual = checked_metadata(root, relative, meta['original_sha256'], meta.get('created_at'))
        before = load_measurements(folder, actual)
        after = deepcopy(before)
        value = after['measurements'].setdefault(str(box_key), {'sections': []})
        records = value['sections']
        found = next((s for s in records if s['id'] == section_id), None)
        if section_id and found is None:
            raise ValueError('所选截面已不存在，请重新选择')
        if delete:
            if found is None:
                raise ValueError('请先选择要删除的截面')
            records.remove(found)
            changed = found
        else:
            if (not points or len(points) != 2 or any(len(p) != 2 for p in points) or
                    any(type(v) not in (int, float) or not math.isfinite(v) for p in points for v in p) or
                    any(not 0 <= p[0] <= actual['size'][0] or not 0 <= p[1] <= actual['size'][1] for p in points)):
                raise ValueError('测量点超出原图或格式无效')
            width = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
            if width == 0:
                raise ValueError('两个点重合，请重新选择裂缝两侧边界')
            if found is None:
                index = value.get('next_label', len(records))
                used = {s['label'] for s in records}
                while section_label(index) in used:
                    index += 1
                found = {'id': uuid.uuid4().hex, 'label': section_label(index)}
                value['next_label'] = index + 1
                records.append(found)
            found.update(endpoints=deepcopy(points), width_px=width, unit='original_image_px',
                         method='manual_boundary_grid', updated_at=datetime.now(timezone.utc).isoformat())
            changed = found
        rebuild_compatibility(value)
        legacy = folder / 'measurements.json'
        if legacy.exists() and read_json(legacy, {}).get('schema_version', 1) < 2:
            backup = folder / 'measurements_legacy.json'
            if not backup.exists():
                backup.write_bytes(legacy.read_bytes())
        atomic_json(legacy, after)
        return before, after, deepcopy(changed)


def undo_measurement(root, relative, meta, before, after):
    with catalog_lock(root):
        folder, actual = checked_metadata(root, relative, meta['original_sha256'], meta.get('created_at'))
        if load_measurements(folder, actual) != after:
            raise ValueError('测量记录已被其他窗口修改，未覆盖这些修改')
        atomic_json(folder / 'measurements.json', before)


def experiment_tags(folder, meta):
    tags = dict(meta.get('conditions', {}))
    extra = read_json(folder / 'experiment.json', {})
    if extra.get('original_sha256') == meta['original_sha256']:
        tags.update(extra.get('tags', {}))
    tags.setdefault('review', 'unknown')
    return tags


def save_tags(root, relative, meta, tags):
    validate_tags(tags)
    with catalog_lock(root):
        folder, _ = checked_metadata(root, relative, meta['original_sha256'], meta.get('created_at'))
        atomic_json(folder / 'experiment.json', {'original_sha256': meta['original_sha256'],
                    'tags': tags, 'updated_at': datetime.now(timezone.utc).isoformat()})


def validate_tags(tags):
    if tags.get('distance_m') is not None and not 0 < tags['distance_m'] <= 1000:
        raise ValueError('距离须大于0且不超过1000 m')
    if tags.get('lux') is not None and not 0 <= tags['lux'] <= 1000000:
        raise ValueError('照度范围为0～1000000 lux')
    if tags.get('light', 'unknown') not in ('unknown', 'bright', 'dark'):
        raise ValueError('光照标签无效')


def tagged_capture_name(folder, meta, tags):
    """Replace generated name fields; retain a user-defined descriptive prefix."""
    specimen = re.sub(r'[^\w.-]+', '_', str(tags.get('specimen_id') or 'crack')).strip('._')[:24] or 'crack'
    distance = '{:g}m'.format(tags['distance_m']) if tags.get('distance_m') is not None else '距离未设'
    light = {'bright': '明', 'dark': '暗'}.get(tags.get('light'), '光照未设')
    current = folder.name
    pattern = r'^(\d{2}-\d{2}-\d{2}_\d{3})_.+_(?:距离未设|\d+(?:\.\d+)?m)_(?:光照未设|明|暗)_(自动|手动|定时)(?:_(\d+))?$'
    match = re.match(pattern, current)
    previous = read_json(folder / 'experiment.json', {})
    if match:
        base = match.group(1)
        ending = '_' + match.group(2) + ('_' + match.group(3) if match.group(3) else '')
    else:
        base = (previous.get('name_base') if previous.get('last_generated_name') == current
                and previous.get('original_sha256') == meta['original_sha256'] else current) or current
        ending = ''
    tail = '_{}_{}_{}{}'.format(specimen, distance, light, ending)
    # Leave room for collision suffixes and all image suffixes on Windows.
    while len((base + tail).encode('utf-8')) > 140 and base:
        base = base[:-1]
    proposed = capture_name(base + tail)
    return proposed, base


def save_tags_and_rename(root, relative, meta, tags):
    validate_tags(tags)
    folder = capture_folder(root, relative)
    name, base = tagged_capture_name(folder, meta, tags)
    update = {'original_sha256': meta['original_sha256'], 'tags': deepcopy(tags),
              'name_base': base, 'updated_at': datetime.now(timezone.utc).isoformat()}
    # Tag update, image filenames and catalog commit share the rename lock/rollback.
    return rename_capture(root, relative, name, meta['original_sha256'], meta.get('created_at'),
                          experiment_update=update, unique_name=True)
