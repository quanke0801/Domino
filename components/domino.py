import logging
logger = logging.getLogger(__name__)

import numpy as np

from Domino.components.component import Component, PointRef, VectorRef
from Domino.geometry.pose import Pose, rotation_matrix_from_rpy
from Domino.geometry.collision import OBB, SAT
from Domino.components.ground import Ground

"""
Default local coordinate system:
  +------------------+
 /                  /|
+------------------+ |
|         z+       | |
|         ^        | |
|         |        | |
|         O--> y+  | |
|        /         | |
|       L          | |
|      x+          | +
|                  |/
+------------------+
"""
class Domino(Component):
    SIZE = np.array([0.015, 0.05, 0.1])
    LEAN_ANGLE = np.arctan(SIZE[0] / SIZE[2]) + 1.0E-2

    def __init__(self, to_parent: Pose | None = None):
        super().__init__(to_parent)
        for sign_x, label_x in [(-1, "x-"), (0, ""), (1, "x+")]:
            for sign_y, label_y in [(-1, "y-"), (0, ""), (1, "y+")]:
                for sign_z, label_z in [(-1, "z-"), (0, ""), (1, "z+")]:
                    anchor_name = f"{label_x}{label_y}{label_z}"
                    anchor_position_local = np.array([sign_x, sign_y, sign_z]) * self.SIZE / 2
                    self.anchors[anchor_name] = anchor_position_local

    @staticmethod
    def from_rpy(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> "Domino":
        rotation = rotation_matrix_from_rpy(roll, pitch, yaw)
        return Domino(Pose(rotation=rotation))

    @staticmethod
    def standing(yaw: float = 0.0) -> "Domino":
        rotation = rotation_matrix_from_rpy(0, 0, yaw)
        domino = Domino(Pose(rotation=rotation))
        domino.add_socket("in", domino)
        domino.add_socket("out", domino)
        return domino

    @staticmethod
    def sideways(yaw: float = 0.0) -> "Domino":
        rotation = rotation_matrix_from_rpy(0, 0, yaw) @ rotation_matrix_from_rpy(np.pi / 2, 0, 0)
        return Domino(Pose(rotation=rotation))

    @staticmethod
    def lying(yaw: float = 0.0) -> "Domino":
        rotation = rotation_matrix_from_rpy(0, 0, yaw) @ rotation_matrix_from_rpy(0, np.pi / 2, 0)
        return Domino(Pose(rotation=rotation))

    def obb_in_world(self) -> OBB:
        return OBB(self.to_world(), Domino.SIZE / 2)
