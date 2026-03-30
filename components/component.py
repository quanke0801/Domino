from dataclasses import dataclass
import logging
logger = logging.getLogger(__name__)

import numpy as np

from Domino.geometry.pose import Pose, rotation_matrix_from_axis_angle, rotation_matrix_from_rpy
from Domino.geometry.collision import OBB, SAT


@dataclass
class PointRef:
    component: "Component | None"
    point: np.ndarray


@dataclass
class VectorRef:
    component: "Component | None"
    vector: np.ndarray


class Component:
    NAME_TO_AXIS = {
        "x+": np.array([1, 0, 0]),
        "x-": np.array([-1, 0, 0]),
        "y+": np.array([0, 1, 0]),
        "y-": np.array([0, -1, 0]),
        "z+": np.array([0, 0, 1]),
        "z-": np.array([0, 0, -1]),
    }

    def __init__(self, to_parent: Pose | None = None):
        self.parent = None
        self.to_parent = to_parent if to_parent is not None else Pose()
        self.children = {}
        self.anchors = {
            "": np.array([0, 0, 0])
        }
        self.sockets = {}
    
    def child(self, name: str) -> "Component":
        return self.children[name]

    def anchor(self, anchor: str | np.ndarray) -> PointRef:
        if isinstance(anchor, str):
            return PointRef(self, self.anchors[anchor])
        elif isinstance(anchor, np.ndarray):
            return PointRef(self, anchor)
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")

    def axis(self, axis: str | np.ndarray) -> VectorRef:
        if isinstance(axis, str):
            return VectorRef(self, Component.NAME_TO_AXIS[axis])
        elif isinstance(axis, np.ndarray):
            return VectorRef(self, axis / np.linalg.norm(axis))
        else:
            raise ValueError(f"unsupported axis type: {type(axis)}")
    
    def add_child(self, name: str, child: "Component") -> None:
        child.parent = self
        self.children[name] = child
    
    def to_world(self) -> Pose:
        if self.parent is None:
            return self.to_parent
        return self.parent.to_world() * self.to_parent
    
    def to_other(self, other: "Component") -> Pose:
        return other.to_world().inverse() * self.to_world() if other is not None else self.to_world()
    
    # These method should not be needed.

    # def orient_abs(self, rotation_in_parent: np.ndarray) -> "Component":
    #     self.to_parent.set_rotation(rotation_in_parent)
    #     return self

    # def orient_like(self, target: "Component") -> "Component":
    #     target_to_world = target.to_world()
    #     parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
    #     self.to_parent.set_rotation((parent_to_world.inverse() * target_to_world).rotation)
    #     return self
    
    # def orient(self, arg: "np.ndarray | Component") -> "Component":
    #     if isinstance(arg, np.ndarray):
    #         return self.orient_abs(arg)
    #     elif isinstance(arg, Component):
    #         return self.orient_like(arg)
    #     else:
    #         raise ValueError(f"unsupported operand type: {type(arg)}")
    
    # def place_abs(self, anchor_name: str, target_in_parent: np.ndarray) -> "Component":
    #     anchor_position_local = self.anchors[anchor_name]
    #     self.to_parent.set_position(np.zeros(3))
    #     anchor_position_in_parent = self.to_parent * anchor_position_local
    #     self.to_parent.set_position(target_in_parent - anchor_position_in_parent)
    #     return self
    
    def place(self, anchor: str, target: PointRef) -> "Component":
        anchor_in_parent = self.to_parent.apply_point(self.anchors[anchor])
        target_to_parent = target.component.to_other(self.parent)
        target_in_parent = target_to_parent.apply_point(target.point)
        self.to_parent.set_position(target_in_parent - anchor_in_parent)
        return self

    def move(self, delta: np.ndarray | VectorRef) -> "Component":
        if isinstance(delta, np.ndarray):
            delta_in_parent = self.to_parent.apply_vector(delta)
        elif isinstance(delta, VectorRef):
            delta_in_parent = delta.component.to_other(self.parent).apply_vector(delta.vector)
        else:
            raise ValueError(f"unsupported delta type: {type(delta)}")
        self.to_parent.set_position(self.to_parent.position + delta_in_parent)
        return self
    
    def rotate(self, anchor: None | str | np.ndarray | PointRef, axis: str | np.ndarray | VectorRef, angle: float) -> "Component":
        # Resolve anchor position in local frame.
        if anchor is None:
            anchor_local = np.array([0, 0, 0])
        elif isinstance(anchor, str):
            anchor_local = self.anchors[anchor]
        elif isinstance(anchor, np.ndarray):
            anchor_local = anchor
        elif isinstance(anchor, PointRef):
            anchor_local = anchor.component.to_other(self).apply_point(anchor.point)
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")
        # Resolve axis direction in local frame.
        if isinstance(axis, str):
            axis_local = Component.NAME_TO_AXIS[axis]
        elif isinstance(axis, np.ndarray):
            axis_local = axis / np.linalg.norm(axis)
        elif isinstance(axis, VectorRef):
            axis_local = axis.component.to_other(self).apply_vector(axis.vector)
        else:
            raise ValueError(f"unsupported axis type: {type(axis)}")
        translation = Pose(position=anchor_local)
        rotation = Pose(rotation=rotation_matrix_from_axis_angle(axis_local, angle))
        self.to_parent = self.to_parent * translation * rotation * translation.inverse()
        return self

    def collect_leaves(self) -> list["Component"]:
        leaves = []
        if len(self.children) == 0:
            leaves.append(self)
        else:
            for child in self.children.values():
                leaves.extend(child.collect_leaves())
        return leaves


class Ground(Component):
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def to_world(self) -> Pose:
        return Pose(position=np.array([0, 0, -1]))
    
    def obb_in_world(self) -> OBB:
        return OBB(self.to_world(), np.array([100, 100, 1]))
    
    def collect_leaves(self) -> list["Component"]:
        return [self]


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
    
    def move_to_touch(self, direction: np.ndarray | VectorRef, target: Component | list[Component]) -> "Domino":
        # Resolve direction in world frame.
        if isinstance(direction, np.ndarray):
            direction_in_world = direction / np.linalg.norm(direction)
        elif isinstance(direction, VectorRef):
            direction_in_world = direction.component.to_world().apply_vector(direction.vector)
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
            anchor_local = anchor.component.to_other(self).apply_point(anchor.point)
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")
        anchor_in_world = self.to_world().apply_point(anchor_local)
        # Resolve axis direction in world frame.
        if isinstance(axis, str):
            axis_local = Component.NAME_TO_AXIS[axis]
        elif isinstance(axis, np.ndarray):
            axis_local = axis / np.linalg.norm(axis)
        elif isinstance(axis, VectorRef):
            axis_local = axis.component.to_other(self).apply_vector(axis.vector)
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


class PileDomino(Component):
    def __init__(self, pile_count: int):
        super().__init__()
        if pile_count <= 0:
            raise ValueError(f"pile_count must be positive, got {pile_count}")
        self.pile_count = pile_count
        self.size = np.array([Domino.SIZE[2], Domino.SIZE[1], Domino.SIZE[0] * pile_count])
        self.add_child("0", Domino().orient(Domino.lying()).place_abs("x+", np.array([0, 0, 0])))
        for i in range(1, pile_count):
            self.add_child(
                f"{i}",
                Domino().orient(Domino.lying()).place_snap("x+", AnchorRef(self.children[f"{i - 1}"], "x-"))
            )
        for sign_x, label_x in [(-1, "x-"), (0, ""), (1, "x+")]:
            for sign_y, label_y in [(-1, "y-"), (0, ""), (1, "y+")]:
                for sign_z, label_z in [(-1, "z-"), (0, ""), (1, "z+")]:
                    anchor_name = f"{label_x}{label_y}{label_z}"
                    anchor_position_local = np.array([sign_x, sign_y, sign_z]) * self.size / 2
                    self.anchors[anchor_name] = anchor_position_local

