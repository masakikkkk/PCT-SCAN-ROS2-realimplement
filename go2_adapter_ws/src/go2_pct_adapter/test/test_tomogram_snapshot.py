"""Exercise the real PCT snapshot loader without loading native libraries."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest


@pytest.fixture
def planner(monkeypatch):
    lib = ModuleType('pct_planner.lib')
    lib.a_star = lib.ele_planner = lib.traj_opt = NS()
    monkeypatch.setitem(sys.modules, 'pct_planner.lib', lib)
    monkeypatch.setitem(sys.modules, 'pct_planner.utils', ModuleType('pct_planner.utils'))
    root = Path(__file__).resolve().parents[4]
    source = root / 'src/pct/pct_planner/pct_planner/planner_wrapper.py'
    spec = importlib.util.spec_from_file_location('pct_planner._snapshot_regression', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    obj = module.TomogramPlanner.__new__(module.TomogramPlanner)
    obj.initPlanner = lambda *arrays: setattr(obj, 'native_inputs', arrays)
    return obj


def metadata():
    data = np.zeros((5, 1, 5, 5), dtype=np.float32)
    data[4] = 2.0
    return dict(data=data, center=[3., 4.], resolution=0.2,
                slice_h0=0.5, slice_dh=0.5)


def test_in_memory_loader_preserves_nan_ground_validity_and_owns_data(planner):
    meta = metadata()
    meta['data'][3, 0, 0, 0] = np.nan
    planner.loadTomogramData(meta)
    assert not planner.elev_g_valid[0, 0, 0]
    assert planner.n_slice == 1 and planner.map_dim == [5, 5]
    meta['data'][0] = 50
    assert planner.trav[0, 2, 2] == 0


@pytest.mark.parametrize('key,value', [('resolution', 0), ('center', [1]),
                                     ('slice_dh', float('nan')),
                                     ('data', np.zeros((5, 0, 5, 5)))])
def test_invalid_snapshot_geometry_rejected(planner, key, value):
    meta = metadata()
    meta[key] = value
    with pytest.raises(ValueError):
        planner.loadTomogramData(meta)


def test_legacy_scene_pickles_still_load(planner):
    root = Path(__file__).resolve().parents[4]
    for scene, name in [('building', 'building2_9_ros2'), ('plaza', 'plaza3_10_ros2')]:
        planner.loadTomogram(tomogram_path=str(root / 'assets' / scene / 'tomogram' / (name + '.pickle')))
        assert planner.n_slice > 0 and np.all(np.isfinite(planner.trav))
