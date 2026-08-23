from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OneSidedRamp:
    height_m: float
    start_x_m: float
    up_ramp_length_m: float
    deck_length_m: float
    down_ramp_length_m: float
    track_center_y_m: float
    track_width_m: float

    def __post_init__(self) -> None:
        positive = (
            self.height_m,
            self.up_ramp_length_m,
            self.deck_length_m,
            self.down_ramp_length_m,
            self.track_width_m,
        )
        if not np.isfinite((*positive, self.start_x_m, self.track_center_y_m)).all():
            raise ValueError("Ramp parameters must be finite")
        if any(value <= 0.0 for value in positive):
            raise ValueError("Ramp dimensions must be positive")

    @property
    def end_x_m(self) -> float:
        return (
            self.start_x_m
            + self.up_ramp_length_m
            + self.deck_length_m
            + self.down_ramp_length_m
        )

    def height(self, x_m, y_m):
        x, y = np.broadcast_arrays(np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float))
        z = np.zeros_like(x)
        y_inside = np.abs(y - self.track_center_y_m) <= 0.5 * self.track_width_m
        up_end = self.start_x_m + self.up_ramp_length_m
        deck_end = up_end + self.deck_length_m
        down_end = deck_end + self.down_ramp_length_m
        up = y_inside & (x >= self.start_x_m) & (x < up_end)
        deck = y_inside & (x >= up_end) & (x <= deck_end)
        down = y_inside & (x > deck_end) & (x <= down_end)
        z[up] = self.height_m * (x[up] - self.start_x_m) / self.up_ramp_length_m
        z[deck] = self.height_m
        z[down] = self.height_m * (down_end - x[down]) / self.down_ramp_length_m
        return z


@dataclass(frozen=True)
class OneSidedRampSequence:
    """Known terrain formed from one or two non-overlapping right-track ramps."""

    first: OneSidedRamp
    second: OneSidedRamp | None = None

    def __post_init__(self) -> None:
        if self.second is not None and self.second.start_x_m < self.first.end_x_m:
            raise ValueError("Second ramp must begin at or after the first ramp ends")

    @property
    def end_x_m(self) -> float:
        return self.first.end_x_m if self.second is None else self.second.end_x_m

    def height(self, x_m, y_m):
        height = self.first.height(x_m, y_m)
        if self.second is not None:
            height = np.maximum(height, self.second.height(x_m, y_m))
        return height
