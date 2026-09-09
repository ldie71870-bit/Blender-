"""Read optional worker progress without relying on hot-reloaded job APIs.

Old contour_jobs modules can survive an extension update in sys.modules. Keep
their running jobs/cleanup handlers intact: the UI reads this side channel on
its own and never treats a missing or incomplete progress file as job failure.
"""
import json
import math


BUCKETS = (
    ('建立', .02, .05), ('地面', .07, .18), ('层', .25, .12),
    ('轮廓', .37, .1), ('井字', .47, .08), ('表面', .55, .12),
    ('细部', .67, .1), ('门洞', .77, .08), ('相机', .85, .1),
)


def _finite(value, default):
    try:
        number = float(value)
        return number if math.isfinite(number) else default
    except (TypeError, ValueError, OverflowError):
        return default


def read(job):
    """One consistent snapshot; progress stays monotonic and below completion."""
    previous = max(0., min(.97, _finite(job.get('fraction'), 0.)))
    stage = job.get('progress_stage', '正在加载场景快照')
    if not isinstance(stage, str) or not stage:
        stage = '正在加载场景快照'
    try:
        data = json.loads((job['root'] / 'progress.json').read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            return stage, previous
        label = data.get('stage')
        if isinstance(label, str) and label:
            stage = label
        current = _finite(data.get('current'), None)
        total = _finite(data.get('total'), None)
        if current is not None and total is not None and total > 0:
            base, span = next(((base, span) for word, base, span in BUCKETS if word in stage), (.05, .02))
            value = base + span * max(0., min(1., current / total))
            previous = max(previous, min(.97, value))
    except (OSError, ValueError, TypeError, AttributeError, KeyError):
        pass
    job['progress_stage'] = stage
    job['fraction'] = previous
    return stage, previous
