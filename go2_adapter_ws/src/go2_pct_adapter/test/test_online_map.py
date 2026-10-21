"""Real CPU tomography and PointCloud2 layout tests; no ROS installation."""

import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest

from go2_pct_adapter.online_map import (build_tomogram, cloud_xyz,
                                       transform_points, validate_settings)


def room(hole=False, ceiling=True):
    axis = np.linspace(-2, 2, 41)
    x, y = np.meshgrid(axis, axis)
    xy = np.column_stack((x.reshape(-1), y.reshape(-1)))
    if hole:
        xy = xy[np.any(np.abs(xy) > 0.3, axis=1)]
    floor = np.column_stack((xy, np.zeros(len(xy))))
    roof = np.column_stack((xy, np.full(len(xy), 2.0)))
    return np.concatenate((floor, roof)) if ceiling else floor


def cpu_settings(**changes):
    result = dict(tomogram_backend="cpu", map_min_points=1)
    result.update(changes)
    return result


def test_dense_room_builds_traversable_ground_without_pickle():
    meta = build_tomogram(room(), cpu_settings())
    data = meta['data']
    x, y = np.array(data.shape[2:]) // 2
    assert data.shape[0] == 5
    assert np.any(data[0, :, x, y] < 20)
    assert np.all(data[3, :, x, y] == 0)
    assert np.all(data[4, :, x, y] == 2)
    assert np.all(np.isfinite(data[:3]))
    assert meta['backend'] == 'cpu'


def test_unobserved_hole_and_body_buffer_are_blocked():
    meta = build_tomogram(room(hole=True), cpu_settings())
    data = meta['data']
    x, y = np.array(data.shape[2:]) // 2
    assert np.all(np.isnan(data[3, :, x, y]))
    assert np.all(data[0, :, x, y] == 50)
    assert np.all(data[0, :, x+1, y] == 50)


def test_missing_ceiling_is_conservative_by_default():
    points = room(ceiling=False)
    conservative = build_tomogram(points, cpu_settings())
    assert np.all(conservative['data'][0] == 50)
    permissive = build_tomogram(points, cpu_settings(require_ceiling_observation=False))
    assert np.any(permissive['data'][0] < 20)


def test_cell_budget_is_checked_before_allocating_map():
    with pytest.raises(ValueError, match='map_max_cells'):
        build_tomogram(room(), cpu_settings(map_max_cells=10))


def test_auto_backend_falls_back_when_cupy_is_unavailable(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'cupy', None)
    meta = build_tomogram(room(), cpu_settings(tomogram_backend='auto'))
    assert meta['backend'] == 'cpu' and meta['backend_fallback']


@pytest.mark.parametrize('settings', [dict(resolution=0), dict(slice_height=1.0),
                                    dict(slope_max=float('nan')), dict(kernel_size=4),
                                    dict(map_max_points=True), dict(inflation=-1),
                                    dict(cost_barrier=10)])
def test_invalid_online_settings_rejected(settings):
    with pytest.raises(ValueError):
        validate_settings(settings)


def cloud(endian='<'):
    # Two organized rows, each with one 24-byte XYZ point plus 8 padding bytes.
    data = b''.join(struct.pack(endian + 'ddd', *p) + b'padding!'
                    for p in [(1, 2, 3), (4, 5, 6)])
    return NS(width=1, height=2, point_step=24, row_step=32, data=data,
              is_bigendian=endian == '>',
              fields=[NS(name=n, offset=i*8, datatype=8, count=1)
                      for i, n in enumerate('xyz')])


@pytest.mark.parametrize('endian', ['<', '>'])
def test_organized_padded_float64_map(endian):
    np.testing.assert_allclose(cloud_xyz(cloud(endian), 10), [[1, 2, 3], [4, 5, 6]])


def test_bad_cloud_layout_and_limit_are_rejected():
    with pytest.raises(ValueError):
        cloud_xyz(cloud(), 1)
    msg = cloud()
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match='Truncated'):
        cloud_xyz(msg, 10)
    msg = cloud()
    msg.fields[0].datatype = 6
    with pytest.raises(ValueError, match='FLOAT32'):
        cloud_xyz(msg, 10)


def test_map_tf_applies_rotation_and_translation():
    value = transform_points(np.array([[1, 0, 0]]), (2, 3, 4),
                             (0, 0, np.sqrt(0.5), np.sqrt(0.5)))
    np.testing.assert_allclose(value, [[2, 4, 4]], atol=1e-6)
