#!/usr/bin/env python3
"""Read-only prototype log scoring. Labels never enter event_core/event_node."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', type=Path, help='Replay artifact prefix, without _node.log')
    parser.add_argument('--labels', type=Path, help='Optional frozen evaluator-only point labels')
    args = parser.parse_args()
    frames, candidates, published = {}, {}, []
    config = None
    text = Path(str(args.prefix) + '_node.log').read_text()
    for line in text.splitlines():
        match = re.search(r'POINT_EVENT_(FRAME|CANDIDATE)\s+(\d+)\s+(\{.*\})', line)
        if match:
            kind, stamp, value = match.groups()
            stamp, value = int(stamp), json.loads(value)
            if kind == 'FRAME':
                frames[stamp] = value
            else:
                candidates.setdefault(stamp, []).append(value)
                if value['reason'] == 'publish':
                    published.append(dict(stamp=stamp, **value))
        match = re.search(r'POINT_EVENT_CONFIG\s+(\{.*\})', line)
        if match:
            config = json.loads(match.group(1))
    assert config is not None and frames, 'Not a completed prototype log'
    ordinals = {stamp:index for index,stamp in enumerate(sorted(frames))}
    cadence = float(np.median(np.diff(sorted(frames)))) if len(frames)>1 else 0
    spans, current = [], {}
    for value in published:
        identifier, stamp = value['track'], value['stamp']
        previous = current.get(identifier)
        if previous is None or ordinals[stamp] != ordinals[previous['last_stamp']] + 1 or \
                (cadence and stamp - previous['last_stamp'] > 1.5 * cadence):
            previous = dict(track=identifier, first_stamp=stamp, last_stamp=stamp, frames=0,
                            event_sources=Counter(), observations_min=3)
            current[identifier] = previous
            spans.append(previous)
        previous['last_stamp'] = stamp
        previous['frames'] += 1
        previous['observations_min'] = min(previous['observations_min'], value['observations'])
        previous['event_sources'].update(dict(zip(['crossing', 'toward', 'away'], value['event_counts'])))
    for span in spans:
        span['duration_seconds'] = (span['last_stamp'] - span['first_stamp'])/1e9
        span['event_sources'] = dict(span['event_sources'])
    labels = []
    if args.labels:
        for row in json.loads(args.labels.read_text()):
            if row['points'] < 6:
                continue
            nearby = [q for q in candidates.get(row['stamp'], []) if np.linalg.norm(np.array(q['center'])-row['center']) < .4]
            geometry_hit = any(q['reason']=='publish' and max(q['size'])<.65 and
                               np.linalg.norm(np.array(q['center'])-row['center'])<.25
                               for q in candidates.get(row['stamp'],[]))
            labels.append(dict(stamp=row['stamp'], prototype_frame=row['stamp'] in frames,
                               published_candidate_matches_frozen_geometry=geometry_hit,
                               nearby_candidates=nearby, reason='no_callback' if row['stamp'] not in frames else
                               'candidate_missing' if not nearby else 'nearby_candidate_published' if any(q['reason']=='publish' for q in nearby) else nearby[0]['reason']))
    result = dict(config=config, frame_count=len(frames),
                  decisions=dict(Counter(q['reason'] for rows in candidates.values() for q in rows)),
                  event_totals={kind:sum(row[kind] for row in frames.values()) for kind in ['events','crossing','toward','away','static']},
                  published_rows=published,published_spans=spans,
                  frozen_label_diagnostics=labels,
                  semantics='Every output has a current measured region. Nearby candidate reasons use a diagnostic 0.4m gate; the separate frozen geometry flag uses 0.25m/0.65m but does not prove bag receipt. Success rates must come from actual bag scoring. Continuous spans break on any unpublished callback or timestamp gap. Box counts remain a proxy, not true false-positive labels.')
    output = Path(str(args.prefix) + '_event_diagnostics.json')
    with output.open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(dict(artifact=str(output), frames=len(frames), published=len(published), decisions=result['decisions'], spans=len(spans))))


if __name__ == '__main__':
    main()
