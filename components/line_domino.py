import numpy as np

from Domino.components.component import Component, PointRef, SocketRef
from Domino.components.domino import Domino


class LineDomino(Component):
    DEFAULT_GAP = Domino.SIZE[2] * 0.75
    
    def __init__(self, start: PointRef, end: PointRef, include: tuple[bool, bool] = (True, True), gap_ratio: float = 1.0, fishbone_angle: float = 0.0):
        super().__init__()
        # Resolve start and end points in local frame.
        start_local = start.to_other(self)[:2]
        end_local = end.to_other(self)[:2]
        # Resolve gap and count.
        length = np.linalg.norm(end_local - start_local)
        yaw = np.arctan2(end_local[1] - start_local[1], end_local[0] - start_local[0])
        expected_gap = gap_ratio * self.DEFAULT_GAP
        gap_count = int(np.ceil(length / expected_gap))
        gap = length / gap_count
        if gap_count <= 1 and not include[0] and not include[1]:
            raise ValueError(f"length {length} is too short to create a line domino with gap ratio {gap_ratio}")
        # Place dominoes.
        indices = list(range(1, gap_count))
        if include[0]:
            indices.insert(0, 0)
        if include[1]:
            indices.append(gap_count)
        for index in indices:
            position_2d = start_local + (end_local - start_local) * index / gap_count
            position_3d = np.array([position_2d[0], position_2d[1], 0])
            delta_yaw = 0
            if index != 0 and index != gap_count:
                delta_yaw = fishbone_angle if index % 2 == 0 else -fishbone_angle
            self.add_child(f"{index}", (
                Domino().standing()
                .rotate("", np.array([0, 0, 1]), yaw + delta_yaw)
                .place("z-", self.anchor(position_3d))
            ))
        self.add_socket("in", self.child(f"{indices[0]}").socket("in"))
        self.add_socket("out", self.child(f"{indices[-1]}").socket("out"))

    @staticmethod
    def from_socket(socket: SocketRef, length: float, gap_ratio: float = 1.0) -> "LineDomino":
        start_point = PointRef(socket.component, socket.position)
        end_point = PointRef(socket.component, socket.position + socket.direction * length)
        return LineDomino(start_point, end_point, [False, True], gap_ratio)

    @staticmethod
    def from_domino(domino: Domino, length: float, gap_ratio: float = 1.0) -> "LineDomino":
        return LineDomino.from_socket(domino.socket("out"), length, gap_ratio)
    
    @staticmethod
    def to_socket(socket: SocketRef, length: float, gap_ratio: float = 1.0) -> "LineDomino":
        start_point = PointRef(socket.component, socket.position - socket.direction * length)
        end_point = PointRef(socket.component, socket.position)
        return LineDomino(start_point, end_point, [True, False], gap_ratio)
    
    @staticmethod
    def to_domino(domino: Domino, length: float, gap_ratio: float = 1.0) -> "LineDomino":
        return LineDomino.to_socket(domino.socket("in"), length, gap_ratio)

    def trigger(self) -> "LineDomino":
        first = self.child("0")
        first.rotate("x+z-", first.axis("y+"), Domino.LEAN_ANGLE)
        return self
