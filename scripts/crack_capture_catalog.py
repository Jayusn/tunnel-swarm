"""Shared catalog updates and strictly scoped edits of one saved image group."""
from copy import deepcopy
from contextlib import contextmanager
from datetime import datetime
import hashlib
import os
from pathlib import Path, PurePosixPath
import shutil
import uuid

from crack_live_common import atomic_json, read_json


@contextmanager
def catalog_lock(root):
    """Serialize recorder/viewer read-modify-write operations across processes."""
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.catalog.lock').open('a+b') as handle:
        if handle.tell() == 0:
            handle.write(b'\0')
            handle.flush()
        handle.seek(0)
        if os.name == 'nt':
            import msvcrt
            msvcrt.locking(handle.fileno(), msvcrt.LK_LOCK, 1)
        else:
            import fcntl
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            handle.seek(0)
            if os.name == 'nt':
                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def capture_folder(root, relative):
    """Only accept a flat capture below a date/session, never root or a link."""
    root = Path(root).resolve()
    if not isinstance(relative, str) or '\\' in relative:
        raise ValueError('图片组路径无效')
    parts = PurePosixPath(relative)
    if parts.is_absolute() or len(parts.parts) != 2 or any(
            part in ('.', '..') or part.startswith('.') for part in parts.parts):
        raise ValueError('只能操作日期或采集轮次下面的单个图片组')
    candidate = root.joinpath(*parts.parts)
    resolved = candidate.resolve()
    if (root not in resolved.parents or resolved != candidate or
            candidate.is_symlink() or candidate.parent.is_symlink()):
        raise ValueError('图片组路径包含链接或超出保存目录')
    return resolved


def _catalog(root):
    path = Path(root) / 'catalog.json'
    value = read_json(path, {'events': []}) if not path.exists() else read_json(path)
    if not isinstance(value, dict) or not isinstance(value.get('events'), list):
        raise ValueError('图片索引无法读取，未修改图片组')
    if any(not isinstance(entry, dict) or not isinstance(entry.get('path'), str)
           for entry in value['events']):
        raise ValueError('图片索引内容无效，未修改图片组')
    return value


def append_capture(root, item):
    with catalog_lock(root):
        folder = capture_folder(root, item['path'])
        # A viewer may have deleted a committed group just before its catalog
        # append. Never resurrect that group in a concurrent writer's index.
        metadata = read_json(folder / 'capture.json')
        if not metadata or metadata.get('created_at') != item.get('created_at'):
            return False
        catalog = _catalog(root)
        events = [entry for entry in catalog['events'] if entry.get('path') != item['path']]
        catalog['events'] = (events + [item])[-200:]
        atomic_json(Path(root) / 'catalog.json', catalog)
        return True


def checked_metadata(root, relative, original_hash, created_at=None):
    folder = capture_folder(root, relative)
    marker = folder / 'capture.json'
    if marker.is_symlink() or not marker.is_file():
        raise ValueError('当前图片组已不存在或尚未保存完成')
    metadata = read_json(marker)
    if (not isinstance(metadata, dict) or not original_hash or
            metadata.get('original_sha256') != original_hash or
            (created_at is not None and metadata.get('created_at') != created_at)):
        raise ValueError('图片组与当前显示内容不一致，请重新选择后再操作')
    return folder, metadata


def delete_capture(root, relative, original_hash, created_at=None):
    """Permanently remove one displayed capture, retaining concurrent new saves."""
    root = Path(root).resolve()
    with catalog_lock(root):
        folder, metadata = checked_metadata(root, relative, original_hash, created_at)
        children = list(folder.iterdir())
        if any(child.is_symlink() or not child.is_file() for child in children):
            raise ValueError('本组含有链接或子目录，未删除；请先检查本组文件')
        original_name = metadata.get('original')
        if not isinstance(original_name, str) or Path(original_name).name != original_name:
            raise ValueError('原图文件名无效，未删除')
        original = folder / original_name
        if hashlib.sha256(original.read_bytes()).hexdigest() != original_hash:
            raise ValueError('原图内容已变化，未删除')
        catalog = _catalog(root)
        catalog['events'] = [entry for entry in catalog['events']
                             if entry.get('path') != PurePosixPath(relative).as_posix()]
        sizes = [child.stat().st_size for child in children]
        staged = folder.with_name('.deleting_' + uuid.uuid4().hex)
        # Validate both resolved absolute targets immediately before the move.
        if (folder.parent != staged.parent or root not in staged.resolve().parents
                or staged.resolve() == root or staged.exists()):
            raise ValueError('删除目标校验失败，未删除')
        folder.rename(staged)
        try:
            atomic_json(root / 'catalog.json', catalog)
        except BaseException:
            staged.rename(folder)
            raise
    # The slow filesystem cleanup does not block a recorder's catalog append.
    if root not in staged.resolve().parents or not staged.name.startswith('.deleting_'):
        raise ValueError('清理路径校验失败')
    try:
        shutil.rmtree(staged)
    except OSError as error:
        raise OSError('本组已移出列表，部分文件未能清理：{}；{}'.format(staged, error)) from error
    return {'path': relative, 'files': len(children), 'bytes': sum(sizes)}


def capture_name(value):
    """Validate one portable folder name without silently changing user input."""
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError('名称不能为空，开头和结尾不能有空格')
    if (value.startswith('.') or value.endswith('.') or
            any(ord(c) < 32 or ord(c) == 127 or c in '<>:"/\\|?*' for c in value)):
        raise ValueError('名称不能以点开头或结尾，也不能含路径分隔符或 <>:"/\\|?* 等字符')
    reserved = {'CON', 'PRN', 'AUX', 'NUL'} | {
        prefix + str(n) for prefix in ('COM', 'LPT') for n in range(1, 10)}
    if value.split('.')[0].upper() in reserved:
        raise ValueError('该名称是系统保留名称，请换一个')
    if len(value.encode('utf-8')) > 160:
        raise ValueError('名称太长，请控制在约50个汉字以内')
    return value


def rename_capture(root, relative, new_name, original_hash, created_at=None,
                   experiment_update=None, unique_name=False):
    """Rename a whole group and its image references under the shared lock.

    The hidden staging folder keeps partially renamed images out of history
    scans. A failed file/metadata/catalog write restores the original group.
    Original pixels and measurements are never re-encoded or rewritten.
    """
    root = Path(root).resolve()
    new_name = capture_name(new_name)
    with catalog_lock(root):
        folder, metadata = checked_metadata(root, relative, original_hash, created_at)
        new_relative = (PurePosixPath(relative).parent / new_name).as_posix()
        destination = capture_folder(root, new_relative)
        base_name, suffix = new_name, 2
        while unique_name and destination.exists() and new_relative != relative:
            new_name = capture_name(base_name + '_{:02d}'.format(suffix))
            new_relative = (PurePosixPath(relative).parent / new_name).as_posix()
            destination = capture_folder(root, new_relative)
            suffix += 1
        if experiment_update is not None:
            if (not isinstance(experiment_update, dict) or
                    experiment_update.get('original_sha256') != original_hash):
                raise ValueError('实验标签与当前图片组不一致')
            experiment_update = deepcopy(experiment_update)
            experiment_update['last_generated_name'] = new_name
            if (folder / 'experiment.json').is_symlink():
                raise ValueError('实验标签文件是链接，未修改')
        if new_relative == relative:
            if experiment_update is not None:
                atomic_json(folder / 'experiment.json', experiment_update)
            return {'path': relative, 'metadata': metadata, 'changed': False}
        if destination.exists():
            raise ValueError('同一日期或轮次下已有这个名称，请换一个；不会覆盖已有照片')
        children = list(folder.iterdir())
        if any(child.is_symlink() or not child.is_file() for child in children):
            raise ValueError('本组含有链接或子目录，未重命名；请先检查本组文件')
        updated = deepcopy(metadata)
        mapping = {}

        def image_name(old, suffix):
            if (not isinstance(old, str) or not old or '/' in old or '\\' in old or
                    Path(old).name != old or not (folder / old).is_file()):
                raise ValueError('图片文件名或文件无效，未重命名')
            target = new_name + suffix + Path(old).suffix
            if old in mapping or target in mapping.values():
                raise ValueError('图片引用有重复，未重命名')
            mapping[old] = target
            return target

        updated['original'] = image_name(metadata.get('original'), '_原图')
        updated['annotated'] = image_name(metadata.get('annotated'), '_检测图')
        for number, box in enumerate(updated.get('boxes', []), 1):
            box['crop'] = image_name(box.get('crop'), '_裂缝{:03d}'.format(number))
        if hashlib.sha256((folder / metadata['original']).read_bytes()).hexdigest() != original_hash:
            raise ValueError('原图内容已变化，未重命名')
        untouched = {child.name for child in children} - set(mapping)
        if set(mapping.values()) & untouched:
            raise ValueError('新图片名与组内其他文件冲突，未重命名')
        updated['capture_name'] = new_name
        updated.setdefault('initial_capture_name', folder.name)
        updated['renamed_at'] = datetime.now().astimezone().isoformat()
        catalog = _catalog(root)
        replacement = entry_from_metadata(new_relative, updated)
        found = False
        events = []
        for entry in catalog['events']:
            if entry['path'] == relative:
                events.append(replacement)
                found = True
            else:
                events.append(entry)
        # Also announce a renamed old group that has fallen out of the recent
        # catalog; viewers still order it by the unchanged capture timestamp.
        if not found:
            events.append(replacement)
        catalog['events'] = events[-200:]
        staged = folder.with_name('.renaming_' + uuid.uuid4().hex)
        if (staged.parent != folder.parent or staged.exists() or
                root not in staged.resolve().parents or destination.parent != folder.parent):
            raise ValueError('重命名目标校验失败，未修改图片组')
        marker_bytes = (folder / 'capture.json').read_bytes()
        experiment_bytes = ((folder / 'experiment.json').read_bytes()
                            if experiment_update is not None and (folder / 'experiment.json').exists() else None)
        moves = []
        folder.rename(staged)
        location = staged
        try:
            for old, target in mapping.items():
                temporary = '.rename_' + uuid.uuid4().hex
                (staged / old).rename(staged / temporary)
                moves.append([old, temporary, target])
            for move in moves:
                (staged / move[1]).rename(staged / move[2])
                move[1] = move[2]
            atomic_json(staged / 'capture.json', updated)
            if experiment_update is not None:
                atomic_json(staged / 'experiment.json', experiment_update)
            if destination.exists():
                raise ValueError('目标名称刚被占用，未覆盖已有照片')
            staged.rename(destination)
            location = destination
            atomic_json(root / 'catalog.json', catalog)
        except BaseException as error:
            try:
                if location != staged:
                    location.rename(staged)
                # Undo both rename phases so even overlapping image names can
                # be restored without replacing another original file.
                for move in moves:
                    temporary = '.rollback_' + uuid.uuid4().hex
                    (staged / move[1]).rename(staged / temporary)
                    move[1] = temporary
                for old, temporary, _ in moves:
                    (staged / temporary).rename(staged / old)
                (staged / 'capture.json').write_bytes(marker_bytes)
                if experiment_update is not None:
                    tag_file = staged / 'experiment.json'
                    if experiment_bytes is None:
                        if tag_file.exists():
                            tag_file.unlink()
                    else:
                        tag_file.write_bytes(experiment_bytes)
                staged.rename(folder)
            except BaseException as rollback_error:
                raise OSError('重命名中断，原文件保留在 {}，请勿删除该目录：{}'.format(
                    staged, rollback_error)) from error
            raise
    return {'path': new_relative, 'metadata': updated, 'changed': True}


def entry_from_metadata(relative, metadata):
    conditions = metadata.get('conditions', {})
    return {'path': relative, 'created_at': metadata.get('created_at', ''),
            'size': metadata.get('size'), 'display_name': metadata.get('capture_name'),
            'captured_at': metadata.get('captured_at'),
            'specimen_id': conditions.get('specimen_id'), 'boxes': len(metadata.get('boxes', [])),
            'reason': metadata.get('reason'), 'distance_m': conditions.get('distance_m'),
            'light': conditions.get('light')}


def event_order(event):
    try:
        stamp = event.get('created_at') or event.get('captured_at')
        value = datetime.fromisoformat(stamp.replace('Z', '+00:00')).timestamp()
    except (ValueError, TypeError, AttributeError, OverflowError):
        value = 0
    return value, event.get('path', '')


def scan_captures(root, interrupted=lambda: False):
    """Read committed groups at the two supported levels, including old sessions."""
    root = Path(root).resolve()
    events = []
    for parent in root.iterdir():
        if interrupted():
            break
        if parent.name.startswith('.') or parent.is_symlink() or not parent.is_dir():
            continue
        for folder in parent.iterdir():
            if interrupted():
                return events
            if folder.name.startswith('.') or folder.is_symlink() or not folder.is_dir():
                continue
            marker = folder / 'capture.json'
            if marker.is_symlink():
                continue
            metadata = read_json(marker)
            if not isinstance(metadata, dict) or not metadata.get('original_sha256'):
                continue
            events.append(entry_from_metadata(folder.relative_to(root).as_posix(), metadata))
    return sorted(events, key=event_order)
