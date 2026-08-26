import numpy as np

from kilin_local_terrain_mapping.elevation import (
    elevation_from_points,
    front_roi,
    grid_coordinates,
    retained_window_roi,
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
