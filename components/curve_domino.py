import numpy as np

from Domino.components.component import Component, SocketRef
from Domino.components.domino import Domino


class CubicBezier:
    def __init__(self, control_points: list[np.ndarray]):
        if len(control_points) != 4:
            raise ValueError("CubicBezier can only support 4 control points")
        self.control_points = control_points
    
    @staticmethod
    def from_start_end(start: np.ndarray, start_direction: np.ndarray, end: np.ndarray, end_direction: np.ndarray, weight: float = 0.1) -> "CubicBezier":
        """
        https://poe.com/s/yPcGThMITy9Ai087AU4r
        """
        delta = end - start
        start_direction = start_direction / np.linalg.norm(start_direction)
        end_direction = end_direction / np.linalg.norm(end_direction)
        c = np.dot(start_direction, end_direction)
        D = np.dot(delta, start_direction) + np.dot(delta, end_direction)
        numerator = (1 + 12 * weight) * D
        denominator = 2 * (2 + c) + 36 * weight * (1 + c)
        gamma = numerator / denominator
        start_next = start + gamma * start_direction
        end_previous = end - gamma * end_direction
        return CubicBezier([start, start_next, end_previous, end])

    def evaluate(self, t: float | np.ndarray) -> np.ndarray:
        if isinstance(t, np.ndarray):
            t = t[:, np.newaxis]
        return self.control_points[0] * (1 - t) ** 3 + 3 * self.control_points[1] * t * (1 - t) ** 2 + 3 * self.control_points[2] * t ** 2 * (1 - t) + self.control_points[3] * t ** 3

    def derivative(self, t: float | np.ndarray) -> np.ndarray:
        if isinstance(t, np.ndarray):
            t = t[:, np.newaxis]
        return 3 * (self.control_points[1] - self.control_points[0]) * (1 - t) ** 2 + 6 * (self.control_points[2] - self.control_points[1]) * t * (1 - t) + 3 * (self.control_points[3] - self.control_points[2]) * t ** 2


class CurveDomino(Component):
    DEFAULT_GAP = Domino.SIZE[2] * 0.75

    def __init__(self, start: SocketRef, end: SocketRef, include: tuple[bool, bool] = (False, False), gap_ratio: float = 1.0):
        super().__init__()
        # Resolve bezier curve.
        start_position_in_world, start_direction_in_world = start.to_world()
        end_position_in_world, end_direction_in_world = end.to_world()
        self.bezier = CubicBezier.from_start_end(
            start_position_in_world,
            start_direction_in_world,
            end_position_in_world,
            end_direction_in_world
        )
        # Resolve gap and count.
        desired_gap = gap_ratio * self.DEFAULT_GAP
        # Use 5x here, so that don't need to handle multiple dominoes in one gap.
        straight_length = np.linalg.norm(end_position_in_world - start_position_in_world)
        sample_count = 5 * int(np.ceil(straight_length / desired_gap))
        sample_t = np.linspace(0, 1, sample_count)
        sample_points = self.bezier.evaluate(sample_t)
        sample_points = np.stack([sample_points[:, 0], sample_points[:, 1], np.zeros(sample_count)], axis=1)
        discrete_length = np.sum(np.linalg.norm(sample_points[1:] - sample_points[:-1], axis=1))
        sample_directions = self.bezier.derivative(sample_t)
        sample_directions = sample_directions / np.linalg.norm(sample_directions, axis=1, keepdims=True)
        sample_yaws = np.arctan2(sample_directions[:, 1], sample_directions[:, 0])
        gap_count = int(np.ceil(discrete_length / desired_gap))
        gap = discrete_length / gap_count
        # Place dominoes.
        domino_count = 0
        if include[0]:
            self.add_child(f"{domino_count}", (
                Domino().standing()
                .place("z-", self.anchor(sample_points[0]))
                .rotate("", "z+", sample_yaws[0])
            ))
            domino_count += 1
        budget = 0
        for i in range(1, sample_count - 1):
            segment_length = np.linalg.norm(sample_points[i] - sample_points[i - 1])
            budget += segment_length
            if budget > desired_gap:
                # Lerp position and yaw.
                blend = 1 - (budget - desired_gap) / segment_length
                position = sample_points[i - 1] + blend * (sample_points[i] - sample_points[i - 1])
                yaw = sample_yaws[i - 1] + blend * (sample_yaws[i] - sample_yaws[i - 1])
                self.add_child(f"{domino_count}", (
                    Domino().standing()
                    .place("z-", self.anchor(position))
                    .rotate("", "z+", yaw)
                ))
                budget -= desired_gap
                domino_count += 1
        if include[1]:
            self.add_child(f"{domino_count}", (
                Domino().standing()
                .place("z-", self.anchor(sample_points[-1]))
                .rotate("", "z+", sample_yaws[-1])
            ))
            domino_count += 1
