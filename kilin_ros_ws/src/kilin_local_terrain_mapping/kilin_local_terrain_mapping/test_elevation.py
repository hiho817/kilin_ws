import numpy as np

from kilin_local_terrain_mapping.elevation import (
    elevation_from_points,
    fill_from_temporal_cell_memory,
    front_roi,
    grid_coordinates,
    prune_cell_memory_to_bounds,
    retained_window_roi,
    update_temporal_cell_memory,
    voxel_downsample,
)


def test_median_cell_height_rejects_single_high_outlier():
    xs, ys = grid_coordinates(0.0, 0.0, 0.1, 0.0, 0.05, 0.1)
    points = np.array([[0.01, 0.01, 0.10], [0.02, 0.01, 0.11], [0.03, 0.01, 1.50]])
    elevation, valid, counts = elevation_from_points(points, xs, ys, 0.1, minimum_points=2)
    assert valid[0, 0]
    assert counts[0, 0] == 3
    assert elevation[0, 0] == np.float32(0.11)


def test_sparse_cells_remain_invalid():
    xs, ys = grid_coordinates(0.0, 0.0, 0.1, 0.0, 0.05, 0.1)
    elevation, valid, _ = elevation_from_points(np.array([[0.01, 0.01, 0.1]]), xs, ys, 0.1, minimum_points=2)
    assert not valid.any()
    assert not elevation.any()


def test_aligned_rolling_grid_does_not_shift_with_small_pose_jitter():
    first_x, first_y = grid_coordinates(
        0.001, -0.001, 3.0, 1.0, 1.0, 0.1, align_to_resolution=True
    )
    second_x, second_y = grid_coordinates(
        0.049, -0.049, 3.0, 1.0, 1.0, 0.1, align_to_resolution=True
    )
    shifted_x, shifted_y = grid_coordinates(
        0.101, -0.101, 3.0, 1.0, 1.0, 0.1, align_to_resolution=True
    )

    np.testing.assert_allclose(first_x, second_x)
    np.testing.assert_allclose(first_y, second_y)
    assert shifted_x[0] == first_x[0] + 0.1
    assert shifted_y[0] == first_y[0] - 0.1


def test_front_roi_rejects_rear_side_and_high_points():
    points = np.array([[1.0, 0.0, -0.3], [-0.1, 0.0, -0.3], [1.0, 1.1, -0.3], [1.0, 0.0, 0.4]])
    kept = front_roi(points, 0.2, 3.0, 0.9, -1.0, 0.2)
    np.testing.assert_allclose(kept, [[1.0, 0.0, -0.3]])


def test_retained_observation_survives_occlusion_until_it_leaves_window():
    # This point was initially admitted ahead of the robot.  It remains useful
    # after the robot advances, even though a fresh front-only scan may no
    # longer see the exit ramp.
    points = np.array([[1.20, 0.0, -0.3], [-1.25, 0.0, -0.3], [1.0, 1.2, -0.3]])
    kept = retained_window_roi(points, rear_m=1.0, forward_m=3.0, half_width_m=0.9, margin_m=0.1)
    np.testing.assert_allclose(kept, [[1.20, 0.0, -0.3]])


def test_voxel_downsample_bounds_repeated_scan_memory():
    points = np.array([[0.01, 0.01, 0.0], [0.02, 0.02, 0.01], [0.12, 0.01, 0.0]])
    downsampled = voxel_downsample(points, 0.1)
    assert len(downsampled) == 2


def test_mixed_floor_and_ceiling_cell_is_unknown():
    xs, ys = grid_coordinates(0.0, 0.0, 0.1, 0.0, 0.05, 0.1)
    points = np.array([[0.01, 0.01, -0.30], [0.02, 0.01, 1.80]])
    _, valid, counts = elevation_from_points(
        points, xs, ys, 0.1, minimum_points=2, maximum_vertical_span_m=0.35
    )
    assert counts[0, 0] == 2
    assert not valid[0, 0]


def test_temporal_memory_bridges_short_unknown_period_without_overwriting_conflict():
    xs, ys = grid_coordinates(0.0, 0.0, 0.1, 0.0, 0.05, 0.1)
    elevation = np.full((len(ys), len(xs)), np.nan, dtype=np.float32)
    valid = np.zeros_like(elevation, dtype=bool)
    elevation[0, 0], valid[0, 0] = -0.30, True
    memory = update_temporal_cell_memory(
        {}, xs, ys, elevation, valid, resolution_m=0.1, observed_ns=1_000,
        maximum_height_deviation_m=0.10, replacement_observations=2,
    )
    unknown = np.zeros_like(valid)
    filled, filled_valid, count = fill_from_temporal_cell_memory(
        memory, xs, ys, elevation, unknown, resolution_m=0.1,
        now_ns=1_500, hold_ns=1_000, maximum_age_ns=10_000,
    )
    assert count == 1
    assert filled_valid[0, 0]
    assert filled[0, 0] == np.float32(-0.30)

    conflicting = elevation.copy()
    conflicting[0, 0] = 0.10
    update_temporal_cell_memory(
        memory, xs, ys, conflicting, valid, resolution_m=0.1, observed_ns=2_000,
        maximum_height_deviation_m=0.10, replacement_observations=2,
    )
    filled, _, _ = fill_from_temporal_cell_memory(
        memory, xs, ys, elevation, unknown, resolution_m=0.1,
        now_ns=2_100, hold_ns=1_000, maximum_age_ns=10_000,
    )
    assert filled[0, 0] == np.float32(-0.30)

    update_temporal_cell_memory(
        memory, xs, ys, conflicting, valid, resolution_m=0.1, observed_ns=2_200,
        maximum_height_deviation_m=0.10, replacement_observations=2,
    )
    filled, _, _ = fill_from_temporal_cell_memory(
        memory, xs, ys, elevation, unknown, resolution_m=0.1,
        now_ns=2_300, hold_ns=1_000, maximum_age_ns=10_000,
    )
    assert filled[0, 0] == np.float32(0.10)


def test_temporal_memory_expires_instead_of_filling_stale_terrain():
    xs, ys = grid_coordinates(0.0, 0.0, 0.1, 0.0, 0.05, 0.1)
    elevation = np.full((len(ys), len(xs)), np.nan, dtype=np.float32)
    valid = np.zeros_like(elevation, dtype=bool)
    elevation[0, 0], valid[0, 0] = -0.30, True
    memory = update_temporal_cell_memory(
        {}, xs, ys, elevation, valid, resolution_m=0.1, observed_ns=1_000,
        maximum_height_deviation_m=0.10, replacement_observations=2,
    )
    _, output_valid, count = fill_from_temporal_cell_memory(
        memory, xs, ys, elevation, np.zeros_like(valid), resolution_m=0.1,
        now_ns=3_000, hold_ns=1_000, maximum_age_ns=1_500,
    )
    assert count == 0
    assert not output_valid.any()


def test_cell_memory_is_spatially_bounded_to_the_local_window():
    memory = {
        (0.0, 0.0): {"height_m": -0.3, "observed_ns": 1},
        (3.1, 0.0): {"height_m": -0.3, "observed_ns": 1},
        (0.0, 1.1): {"height_m": -0.3, "observed_ns": 1},
    }
    prune_cell_memory_to_bounds(
        memory,
        minimum_x_m=-0.1, maximum_x_m=3.0,
        minimum_y_m=-1.0, maximum_y_m=1.0,
    )
    assert set(memory) == {(0.0, 0.0)}
