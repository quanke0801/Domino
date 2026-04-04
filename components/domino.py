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
        return Domino(Pose(rotation=rotation))

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
    
    def move_to_touch(self, direction: str | np.ndarray | VectorRef, target: Component | list[Component]) -> "Domino":
        # Resolve direction in world frame.
        if isinstance(direction, str):
            direction_in_world = self.axis(direction).to_world()
        elif isinstance(direction, np.ndarray):
            direction_in_world = direction / np.linalg.norm(direction)
        elif isinstance(direction, VectorRef):
            direction_in_world = direction.to_world()
        else:
            raise ValueError(f"unsupported direction type: {type(direction)}")
        # Resolve target OBBs in world frame.
        if isinstance(target, Component):
            target_obbs = [leaf.obb_in_world() for leaf in target.collect_leaves()]
        elif isinstance(target, list[Component]):
            target_obbs = []
            for target_component in target:
                target_obbs.extend([leaf.obb_in_world() for leaf in target_component.collect_leaves()])
        else:
            raise ValueError(f"unsupported target type: {type(target)}")
        # Find contact distance.
        obb = self.obb_in_world()
        contact_distance = None
        for target_obb in target_obbs:
            target_contact_distance = SAT.slide_distance(obb, target_obb, direction_in_world)
            if target_contact_distance is not None and target_contact_distance > 0:
                if contact_distance is None or target_contact_distance < contact_distance:
                    contact_distance = target_contact_distance
        if contact_distance is not None:
            delta_in_world = contact_distance * direction_in_world
            self.move(VectorRef(Ground(), delta_in_world))
        else:
            logger.warning(f"No contact found or already colliding with target {target}.")
        return self
    
    def rotate_to_touch(self, anchor: None | str | np.ndarray | PointRef, axis: str | np.ndarray | VectorRef, target: Component | list[Component]) -> "Domino":
        # Resolve anchor position in world frame.
        if anchor is None:
            anchor_local = np.array([0, 0, 0])
        elif isinstance(anchor, str):
            anchor_local = self.anchors[anchor]
        elif isinstance(anchor, np.ndarray):
            anchor_local = anchor
        elif isinstance(anchor, PointRef):
            anchor_local = anchor.to_other(self)
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")
        anchor_in_world = self.to_world().apply_point(anchor_local)
        # Resolve axis direction in world frame.
        if isinstance(axis, str):
            axis_local = Component.NAME_TO_AXIS[axis]
        elif isinstance(axis, np.ndarray):
            axis_local = axis / np.linalg.norm(axis)
        elif isinstance(axis, VectorRef):
            axis_local = axis.to_other(self)
        else:
            raise ValueError(f"unsupported axis type: {type(axis)}")
        axis_in_world = self.to_world().apply_vector(axis_local)
        # Resolve target OBBs in world frame.
        if isinstance(target, Component):
            target_obbs = [leaf.obb_in_world() for leaf in target.collect_leaves()]
        elif isinstance(target, list[Component]):
            target_obbs = []
            for target_component in target:
                target_obbs.extend([leaf.obb_in_world() for leaf in target_component.collect_leaves()])
        else:
            raise ValueError(f"unsupported target type: {type(target)}")
        # Find contact angle.
        obb = self.obb_in_world()
        rotate_angle = None
        for target_obb in target_obbs:
            target_rotate_angle = SAT.rotate_angle(obb, target_obb, anchor_in_world, axis_in_world)
            if target_rotate_angle is not None:
                if rotate_angle is None or target_rotate_angle < rotate_angle:
                    rotate_angle = target_rotate_angle
        if rotate_angle is not None:
            self.rotate(anchor, axis, rotate_angle)
        else:
            logger.warning(f"No contact found or already colliding with target {target}.")
        return self
