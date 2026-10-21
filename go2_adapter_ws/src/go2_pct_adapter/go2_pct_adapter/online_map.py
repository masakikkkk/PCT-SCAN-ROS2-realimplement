"""ROS-independent conversion of registered map snapshots into PCT data.

Uses this repository's PCT tomography equations; no SLAM or accumulation is
performed here. Each input must already contain the accumulated map.
"""

import math
from types import SimpleNamespace as NS

import numpy as np

from .core import finite, rotate


ONLINE_DEFAULTS = {
    "map_topic": "/uslam/cloud_map",
    "map_timeout_sec": 10.0,
    "map_update_period_sec": 2.0,
    "map_min_points": 500,
    "map_max_points": 2000000,
    "map_max_cells": 2000000,
    "tomogram_backend": "auto",
    "resolution": 0.20,
    "slice_height": 0.50,
    "kernel_size": 5,
    "interval_min": 0.50,
    "interval_free": 0.65,
    "slope_max": 0.40,
    "step_max": 0.20,
    "standable_ratio": 0.20,
    "cost_barrier": 50.0,
    "safe_margin": 0.20,
    "inflation": 0.25,
    # Conservative indoor default: unobserved ceiling is not proof of clearance.
    "require_ceiling_observation": True,
    "goal_snap_radius_m": 0.40,
    "goal_height_tolerance_m": 0.25,
    "goal_reached_radius_m": 0.20,
}


def validate_settings(settings):
    cfg = dict(ONLINE_DEFAULTS)
    cfg.update(settings)
    for name in ("map_timeout_sec", "map_update_period_sec", "resolution",
                 "slice_height", "interval_min", "interval_free", "step_max",
                 "cost_barrier", "goal_snap_radius_m", "goal_height_tolerance_m",
                 "goal_reached_radius_m"):
        if not math.isfinite(float(cfg[name])) or cfg[name] <= 0:
            raise ValueError(name + " must be finite and positive")
    for name in ("map_min_points", "map_max_points", "map_max_cells", "kernel_size"):
        if not isinstance(cfg[name], int) or isinstance(cfg[name], bool) or cfg[name] < 1:
            raise ValueError(name + " must be a positive integer")
    if (cfg["kernel_size"] < 3 or cfg["kernel_size"] % 2 != 1 or
            cfg["map_min_points"] > cfg["map_max_points"] or
            cfg["slice_height"] > cfg["interval_min"] or
            cfg["interval_free"] < cfg["interval_min"]):
        raise ValueError("Invalid kernel, point limits or clearance/slice settings")
    if not finite((cfg["slope_max"], cfg["standable_ratio"])) or not (
            0 < cfg["slope_max"] < math.pi / 2 and 0 < cfg["standable_ratio"] <= 1):
        raise ValueError("Invalid slope or standable ratio")
    for name in ("safe_margin", "inflation"):
        if not math.isfinite(float(cfg[name])) or cfg[name] < 0:
            raise ValueError(name + " must be finite and nonnegative")
    if cfg["tomogram_backend"] not in ("auto", "cpu", "cuda"):
        raise ValueError("tomogram_backend must be auto, cpu or cuda")
    if cfg["cost_barrier"] <= 20.0:
        raise ValueError("cost_barrier must exceed PCT's 20.0 traversability threshold")
    if not isinstance(cfg["require_ceiling_observation"], bool) or not cfg["map_topic"]:
        raise ValueError("Invalid ceiling policy or map topic")
    return cfg


def cloud_xyz(msg, max_points):
    """Read padded/organized, either-endian PointCloud2 XYZ without ROS imports."""
    count = msg.width * msg.height
    if count < 1 or count > max_points or msg.point_step < 12:
        raise ValueError("Map point count/point_step is outside configured limits")
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError("Truncated PointCloud2 or invalid row_step")
    names, formats, offsets = [], [], []
    for name in ("x", "y", "z"):
        fields = [field for field in msg.fields if field.name == name]
        if len(fields) != 1 or fields[0].count != 1 or fields[0].datatype not in (7, 8):
            raise ValueError("Map must have scalar FLOAT32/FLOAT64 XYZ fields")
        field = fields[0]
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > msg.point_step:
            raise ValueError("XYZ field exceeds point_step")
        names.append(name)
        formats.append((">" if msg.is_bigendian else "<") + "f" + str(size))
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets,
                         itemsize=msg.point_step))
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    return np.column_stack([records[name].reshape(-1) for name in names]).astype(np.float32)


def transform_points(points, translation, quaternion):
    if not finite(translation):
        raise ValueError("Non-finite map transform")
    rotation = np.column_stack([rotate(quaternion, axis) for axis in
                               ((1, 0, 0), (0, 1, 0), (0, 0, 1))])
    return (points @ rotation.T + np.asarray(translation)).astype(np.float32)


def build_tomogram(points, settings):
    """Build a bounded full snapshot; CPU fallback requires NumPy and SciPy."""
    cfg = validate_settings(settings)
    from scipy import ndimage
    from tomography.cpu_backend import cpu_point2map

    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) > cfg["map_max_points"]:
        raise ValueError("Invalid or oversized map array")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < cfg["map_min_points"]:
        raise ValueError("Too few finite points in accumulated map")
    lo, hi = points.min(axis=0), points.max(axis=0)
    resolution, dh = cfg["resolution"], cfg["slice_height"]
    dims = [max(5, int(math.ceil(float(hi[i] - lo[i]) / resolution)) + 4)
            for i in (0, 1)]
    layers = max(1, int(math.ceil(float(hi[2] - lo[2]) / dh)))
    cells = dims[0] * dims[1] * layers
    if cells > cfg["map_max_cells"]:
        raise ValueError("Map needs %d cells, above map_max_cells=%d; increase resolution" %
                         (cells, cfg["map_max_cells"]))
    center = (lo[:2].astype(np.float64) + hi[:2]) / 2
    h0 = float(lo[2]) + dh
    runtime = NS(map=NS(resolution=resolution, slice_dh=dh), trav=NS(**{
        name: cfg[name] for name in ("kernel_size", "interval_min", "interval_free",
                                     "slope_max", "step_max", "standable_ratio",
                                     "cost_barrier", "safe_margin", "inflation")}))
    backend = cfg["tomogram_backend"]
    reason = ""
    if backend != "cpu":
        try:
            import cupy as cp
            from tomography.tomogram import Tomogram
            free_bytes, _ = cp.cuda.runtime.memGetInfo()
            if cells * 4 * 6 * 4 > free_bytes:
                raise RuntimeError("Insufficient CUDA memory for bounded map")
        except Exception as exc:
            if backend == "cuda":
                raise RuntimeError("CUDA tomography unavailable: " + str(exc)) from exc
            backend, reason = "cpu", str(exc)
        else:
            backend = "cuda"
    if backend == "cuda":
        generator = Tomogram(runtime)
        generator.initMappingEnv(center, *dims, layers, h0)
        result = generator.point2map(points)
    else:
        result = cpu_point2map(points, center, *dims, layers, h0, runtime, np, ndimage)
    cost, gx, gy, ground, ceiling, timings = result
    cost = np.clip(np.nan_to_num(cost, nan=cfg["cost_barrier"],
                                posinf=cfg["cost_barrier"], neginf=cfg["cost_barrier"]),
                   0, cfg["cost_barrier"])
    observed = np.isfinite(ground)
    if cfg["require_ceiling_observation"]:
        observed &= np.isfinite(ceiling)
    # Unknown cells must not become free via sentinel heights. Keep a body-radius
    # buffer around them as well, including the outer boundary of the map.
    radius = int(math.ceil(cfg["inflation"] / resolution))
    axis = np.arange(-radius, radius + 1)
    disk = ((axis[:, None] * resolution) ** 2 +
            (axis[None, :] * resolution) ** 2 <= cfg["inflation"] ** 2 + 1e-9)
    for layer in range(len(cost)):
        blocked = ndimage.binary_dilation(~observed[layer], structure=disk,
                                         border_value=1)
        cost[layer, blocked] = cfg["cost_barrier"]
    gx = np.zeros_like(cost)
    gy = np.zeros_like(cost)
    gx[:, 1:-1, :] = cost[:, 2:, :] - cost[:, :-2, :]
    gy[:, :, 1:-1] = cost[:, :, 2:] - cost[:, :, :-2]
    return dict(data=np.stack((cost, gx, gy, ground, ceiling)).astype(np.float32),
                resolution=resolution, center=center, slice_h0=h0, slice_dh=dh,
                backend=backend, backend_fallback=reason, input_points=len(points),
                initial_cells=cells, timings=timings)
