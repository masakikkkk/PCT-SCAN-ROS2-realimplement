"""CPU PCT stages shared by offline conversion and online mapping.

Extracted from this repository's scripts/generate_pct_tomogram.py.
"""

import time


def _round_like_c(values, np):
    """Match C/CUDA round(), including half values away from zero."""
    return np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))


def cpu_point2map(points, center, map_dim_x, map_dim_y, n_slice_init, slice_h0,
                  config, np, ndimage):
    """CPU implementation of PCT's three CUDA stages.

    The equations and layer-simplification rule mirror
    ``tomography.tomogram.Tomogram``.  It is slower than CUDA but makes asset
    generation reproducible on ROS workstations without an active GPU driver.
    """
    started = time.perf_counter()
    resolution = config.map.resolution
    slice_dh = config.map.slice_dh
    half_trav = config.trav.kernel_size // 2
    step_stand = 1.2 * resolution * np.tan(config.trav.slope_max)
    step_cross = config.trav.step_max
    standable_th = int(
        config.trav.standable_ratio * (2 * half_trav + 1) ** 2
    ) - 1

    relative = (points[:, :2] - np.asarray(center, dtype=np.float32)) / resolution
    xy = _round_like_c(relative, np).astype(np.int32)
    xy[:, 0] += map_dim_x // 2
    xy[:, 1] += map_dim_y // 2
    inside = (
        (xy[:, 0] >= 0) & (xy[:, 0] < map_dim_x)
        & (xy[:, 1] >= 0) & (xy[:, 1] < map_dim_y)
    )
    xy = xy[inside]
    z = points[inside, 2]
    flat_xy = xy[:, 0] * map_dim_y + xy[:, 1]
    layer_size = map_dim_x * map_dim_y

    layers_g = np.full(
        (n_slice_init, layer_size), -1e6, dtype=np.float32
    )
    layers_c = np.full(
        (n_slice_init, layer_size), 1e6, dtype=np.float32
    )
    for layer in range(n_slice_init):
        slice_height = slice_h0 + layer * slice_dh
        lower = z <= slice_height
        np.maximum.at(layers_g[layer], flat_xy[lower], z[lower])
        np.minimum.at(layers_c[layer], flat_xy[~lower], z[~lower])
    layers_g = layers_g.reshape(n_slice_init, map_dim_x, map_dim_y)
    layers_c = layers_c.reshape(n_slice_init, map_dim_x, map_dim_y)

    grad_mag_sq = np.zeros_like(layers_g)
    grad_mag_max = np.zeros_like(layers_g)
    diff_x_sq = np.maximum(
        (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
        (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2,
    )
    diff_y_sq = np.maximum(
        (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
        (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2,
    )
    grad_mag_sq[:, 1:-1, 1:-1] = (
        diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
    )
    grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
        diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :]
    )
    map_seconds = time.perf_counter() - started

    started_trav = time.perf_counter()
    interval = layers_c - layers_g
    trav_cost = np.maximum(
        0.0, 20.0 * (config.trav.interval_free - interval)
    ).astype(np.float32)
    barrier = interval < config.trav.interval_min
    standable = grad_mag_sq <= step_stand ** 2
    crossable = grad_mag_max <= step_cross ** 2
    neighbourhood = np.ones(
        (1, 2 * half_trav + 1, 2 * half_trav + 1), dtype=np.int16
    )
    standable_count = ndimage.convolve(
        standable.astype(np.int16), neighbourhood, mode="constant", cval=0
    )
    crossing = ~standable
    barrier |= crossing & (~crossable | (standable_count < standable_th))
    trav_cost[standable] += (
        15.0 * grad_mag_sq[standable] / max(step_stand ** 2, 1e-12)
    )
    accepted_crossing = crossing & ~barrier
    trav_cost[accepted_crossing] += (
        20.0 * grad_mag_max[accepted_crossing] / max(step_cross ** 2, 1e-12)
    )
    trav_cost[barrier] = config.trav.cost_barrier

    half_inf = int(
        (config.trav.safe_margin + config.trav.inflation) / resolution
    )
    axis = np.arange(-half_inf, half_inf + 1, dtype=np.float32) * resolution
    distance = np.hypot(axis[:, None], axis[None, :])
    weights = np.clip(
        1.0
        - (distance - config.trav.inflation)
        / (config.trav.safe_margin + resolution),
        0.0,
        1.0,
    )
    footprint = weights > 0.0
    log_structure = np.full(weights.shape, -np.inf, dtype=np.float32)
    log_structure[footprint] = np.log(weights[footprint])
    inflated_cost = np.empty_like(trav_cost)
    for layer in range(n_slice_init):
        log_cost = np.full(trav_cost[layer].shape, -np.inf, dtype=np.float32)
        positive = trav_cost[layer] > 0.0
        log_cost[positive] = np.log(trav_cost[layer][positive])
        dilated = ndimage.grey_dilation(
            log_cost,
            footprint=footprint,
            structure=log_structure,
            mode="constant",
            cval=-np.inf,
        )
        inflated_cost[layer] = np.exp(dilated)
    trav_seconds = time.perf_counter() - started_trav

    started_simp = time.perf_counter()
    idx_simp = [0]
    if n_slice_init > 1:
        lower_idx, middle_idx = 0, 1
        diff_h = layers_g[1:] - layers_g[:-1]
        while middle_idx < n_slice_init - 2:
            unique = (
                (
                    (layers_g[middle_idx] - layers_g[lower_idx] > 0)
                    | (inflated_cost[lower_idx] > inflated_cost[middle_idx])
                )
                & (diff_h[middle_idx] > 0)
                & (inflated_cost[middle_idx] < config.trav.cost_barrier)
            )
            if np.any(unique):
                idx_simp.append(middle_idx)
                lower_idx = middle_idx
            middle_idx += 1
        idx_simp.append(middle_idx)

    selected_cost = inflated_cost[idx_simp]
    selected_ground = layers_g[idx_simp]
    selected_ceiling = layers_c[idx_simp]
    grad_x = np.zeros_like(selected_ground)
    grad_y = np.zeros_like(selected_ground)
    grad_x[:, 1:-1, :] = selected_cost[:, 2:, :] - selected_cost[:, :-2, :]
    grad_y[:, :, 1:-1] = selected_cost[:, :, 2:] - selected_cost[:, :, :-2]
    selected_ground = np.where(selected_ground > -1e6, selected_ground, np.nan)
    selected_ceiling = np.where(selected_ceiling < 1e6, selected_ceiling, np.nan)
    simp_seconds = time.perf_counter() - started_simp
    return (
        selected_cost,
        grad_x,
        grad_y,
        selected_ground,
        selected_ceiling,
        {
            "t_map": map_seconds * 1000.0,
            "t_trav": trav_seconds * 1000.0,
            "t_simp": simp_seconds * 1000.0,
        },
    )
