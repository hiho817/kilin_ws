import numpy as np

from kilin_known_terrain_controller.terrain_window_policy import (
    fill_isolated_unknown_nodes,
    infer_flat_height_ahead,
    seed_initial_flat_support,
)


def test_single_unknown_node_is_filled_only_when_cardinal_neighbours_agree():
    heights = np.zeros((5, 5), dtype=float)
    valid = np.ones((5, 5), dtype=bool)
    valid[2, 2] = False

    output, output_valid, filled = fill_isolated_unknown_nodes(
        heights, valid, maximum_height_span_m=0.02
    )

    assert filled == 1
    assert output_valid[2, 2]
    assert output[2, 2] == 0.0


def test_contiguous_unknown_area_is_not_fabricated():
    heights = np.zeros((5, 5), dtype=float)
    valid = np.ones((5, 5), dtype=bool)
    valid[2, 1:4] = False

    _, output_valid, filled = fill_isolated_unknown_nodes(
        heights, valid, maximum_height_span_m=0.02
    )

    assert filled == 0
    assert not np.any(output_valid[2, 1:4])


def test_initial_flat_support_is_seeded_from_verified_front_strip_only():
    coordinates = np.arange(-1.0, 1.01, 0.1)
    heights = np.full((coordinates.size, coordinates.size), np.nan)
    valid = np.zeros_like(heights, dtype=bool)
    x, y = np.meshgrid(coordinates, coordinates)
    observed = (x >= 0.2) & (x <= 0.8) & (np.abs(y) <= 0.5)
    heights[observed] = -0.28
    valid[observed] = True

    flat_height = infer_flat_height_ahead(
        coordinates, coordinates, heights, valid,
        origin_x_m=0.0, origin_y_m=0.0, yaw_rad=0.0,
        minimum_forward_m=0.2, maximum_forward_m=0.8,
        half_width_m=0.5, maximum_height_span_m=0.02,
    )
    output, output_valid, filled = seed_initial_flat_support(
        coordinates, coordinates, heights, valid,
        origin_x_m=0.0, origin_y_m=0.0, yaw_rad=0.0,
        flat_height_m=flat_height, rear_m=0.65, forward_m=0.75, half_width_m=0.5,
    )

    assert flat_height == -0.28
    assert filled > 0
    assert output_valid[10, 4]  # x=-0.6, y=0: required rear support area
    assert output[10, 4] == -0.28
    assert not output_valid[0, 10]  # lateral outside the support envelope
