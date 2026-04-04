from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from Domino.geometry.pose import Pose, rotation_matrix_from_axis_angle, rotation_matrix_from_rpy
from Domino.geometry.collision import OBB, SAT


@dataclass
class PointRef:
    component: "Component | None"
    point: np.ndarray

    def to_world(self) -> np.ndarray:
        if self.component is None:
            return self.point
        return self.component.to_world().apply_point(self.point)
    
    def to_other(self, other: "Component | None") -> np.ndarray:
        if other is None:
            return self.to_world()
        return other.to_world().inverse().apply_point(self.to_world())


@dataclass
class VectorRef:
    component: "Component | None"
    vector: np.ndarray

    def to_world(self) -> np.ndarray:
        if self.component is None:
            return self.vector
        return self.component.to_world().apply_vector(self.vector)
    
    def to_other(self, other: "Component | None") -> np.ndarray:
        if other is None:
            return self.to_world()
        return other.to_world().inverse().apply_vector(self.to_world())


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
    
    def copy(self) -> "Component":
        return deepcopy(self)
    
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
            return VectorRef(self, axis)
        else:
            raise ValueError(f"unsupported axis type: {type(axis)}")
    
    def add_child(self, name: str, child: "Component") -> None:
        child.parent = self
        self.children[name] = child
    
    def to_world(self) -> Pose:
        if self.parent is None:
            return self.to_parent
        return self.parent.to_world() * self.to_parent
    
    def to_other(self, other: "Component | None") -> Pose:
        if other is None:
            return self.to_world()
        return other.to_world().inverse() * self.to_world()
    
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
        anchor_in_parent = self.to_parent.apply_vector(self.anchors[anchor])
        target_in_parent = target.to_other(self.parent)
        self.to_parent.set_position(target_in_parent - anchor_in_parent)
        return self

    def move(self, delta: np.ndarray | VectorRef) -> "Component":
        if isinstance(delta, np.ndarray):
            delta_in_parent = self.to_parent.apply_vector(delta)
        elif isinstance(delta, VectorRef):
            delta_in_parent = delta.to_other(self.parent)
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
            anchor_local = anchor.to_other(self)
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")
        # Resolve axis direction in local frame.
        if isinstance(axis, str):
            axis_local = Component.NAME_TO_AXIS[axis]
        elif isinstance(axis, np.ndarray):
            axis_local = axis / np.linalg.norm(axis)
        elif isinstance(axis, VectorRef):
            axis_local = axis.to_other(self)
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
