from dataclasses import dataclass
import logging
logger = logging.getLogger(__name__)

import numpy as np

from Domino.geometry.pose import Pose, rotation_matrix_from_axis_angle, rotation_matrix_from_rpy
from Domino.geometry.collision import OBB, SAT


NAME_TO_AXIS = {
    "x+": np.array([1, 0, 0]),
    "x-": np.array([-1, 0, 0]),
    "y+": np.array([0, 1, 0]),
    "y-": np.array([0, -1, 0]),
    "z+": np.array([0, 0, 1]),
    "z-": np.array([0, 0, -1]),
}
GROUND_OBB = OBB(Pose(position = np.array([0, 0, -1])), np.array([100, 100, 1]))


@dataclass
class AnchorRef:
    component: "Component"
    anchor_name: str


class Component:
    def __init__(self, to_parent: Pose | None = None):
        self.parent = None
        self.to_parent = to_parent if to_parent is not None else Pose()
        self.children = {}
        self.anchors = {
            "": np.array([0, 0, 0])
        }
        self.sockets = {}
        # self.aabb = AABB()
    
    def child(self, name: str) -> "Component":
        return self.children[name]
    
    def anchor(self, name: str) -> AnchorRef:
        return AnchorRef(self, name)
    
    def add_child(self, name: str, child: "Component") -> None:
        child.parent = self
        self.children[name] = child
    
    def to_world(self) -> Pose:
        if self.parent is None:
            return self.to_parent
        return self.parent.to_world() * self.to_parent
    
    def orient_abs(self, rotation_in_parent: np.ndarray) -> "Component":
        self.to_parent.set_rotation(rotation_in_parent)
        return self

    def orient_like(self, target: "Component") -> "Component":
        target_to_world = target.to_world()
        parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
        self.to_parent.set_rotation((parent_to_world.inverse() * target_to_world).rotation)
        return self
    
    def orient(self, arg: "np.ndarray | Component") -> "Component":
        if isinstance(arg, np.ndarray):
            return self.orient_abs(arg)
        elif isinstance(arg, Component):
            return self.orient_like(arg)
        else:
            raise ValueError(f"unsupported operand type: {type(arg)}")
    
    def place_abs(self, anchor_name: str, target_in_parent: np.ndarray) -> "Component":
        anchor_position_local = self.anchors[anchor_name]
        self.to_parent.set_position(np.zeros(3))
        anchor_position_in_parent = self.to_parent * anchor_position_local
        self.to_parent.set_position(target_in_parent - anchor_position_in_parent)
        return self
    
    def place_snap(self, anchor_name: str, target_anchor: AnchorRef) -> "Component":
        target, target_anchor_name = target_anchor.component, target_anchor.anchor_name
        target_to_world = target.to_world()
        parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
        target_to_parent = parent_to_world.inverse() * target_to_world
        return self.place_abs(anchor_name, target_to_parent * target.anchors[target_anchor_name])
    
    def move(self, delta_local: np.ndarray) -> "Component":
        self.to_parent.set_position(self.to_parent.position + delta_local)
        return self
    
    def rotate(self, anchor: None | str | np.ndarray | AnchorRef, axis_local: str | np.ndarray, angle: float) -> "Component":
        # Resolve anchor position in local frame.
        anchor_position_local = np.array([0, 0, 0])
        if anchor is None:
            anchor_position_local = np.array([0, 0, 0])
        elif isinstance(anchor, str):
            anchor_position_local = self.anchors[anchor]
        elif isinstance(anchor, np.ndarray):
            anchor_position_local = anchor
        elif isinstance(anchor, AnchorRef):
            anchor_to_world = anchor.component.to_world()
            self_to_world = self.to_world()
            anchor_to_self = anchor_to_world.inverse() * self_to_world
            anchor_position_local = anchor_to_self * anchor.component.anchors[anchor.anchor_name]
        else:
            raise ValueError(f"unsupported anchor type: {type(anchor)}")
        # Resolve axis in local frame.
        if isinstance(axis_local, str):
            axis_local = NAME_TO_AXIS[axis_local]
        elif isinstance(axis_local, np.ndarray):
            axis_local = axis_local / np.linalg.norm(axis_local)
        else:
            raise ValueError(f"unsupported axis type: {type(axis_local)}")
        translation = Pose(position = anchor_position_local)
        rotation = Pose(rotation = rotation_matrix_from_axis_angle(axis_local, angle))
        self.to_parent = self.to_parent * translation * rotation * translation.inverse()
        return self

    def collect_dominoes(self) -> list["Domino"]:
        dominoes = []
        if isinstance(self, Domino):
            dominoes.append(self)
        else:
            for child in self.children.values():
                dominoes.extend(child.collect_dominoes())
        return dominoes

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
    def from_rpy(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
        return rotation_matrix_from_rpy(roll, pitch, yaw)

    @staticmethod
    def standing(yaw: float = 0.0) -> np.ndarray:
        return Domino.from_rpy(0, 0, yaw)

    @staticmethod
    def sideways(yaw: float = 0.0) -> np.ndarray:
        return Domino.from_rpy(0, 0, yaw) @ Domino.from_rpy(np.pi / 2, 0, 0)

    @staticmethod
    def lying(yaw: float = 0.0) -> np.ndarray:
        return Domino.from_rpy(0, 0, yaw) @ Domino.from_rpy(0, np.pi / 2, 0)

    def obb_in_world(self) -> OBB:
        to_world = self.to_world()
        return OBB(to_world, Domino.SIZE / 2)
    
    def move_to_touch(self, axis_in_parent: np.ndarray, target: Component | None) -> "Domino":
        obb = self.obb_in_world()
        parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
        axis_in_parent = axis_in_parent / np.linalg.norm(axis_in_parent)
        axis_in_world = parent_to_world.rotation @ axis_in_parent
        contact_distance = None
        target_obbs = []
        if target is not None:
            target_obbs = [target_domino.obb_in_world() for target_domino in target.collect_dominoes()]
        else:
            target_obbs = [GROUND_OBB]
        for target_obb in target_obbs:
            target_contact_distance = SAT.slide_t(obb, target_obb, axis_in_world)
            if target_contact_distance is not None and target_contact_distance > 0:
                if contact_distance is None or target_contact_distance < contact_distance:
                    contact_distance = target_contact_distance
        if contact_distance is not None:
            self.move(contact_distance * axis_in_parent)
        else:
            logger.warning(f"No contact found or already colliding with target {target}.")
        return self
    
    def rotate_to_touch(self, anchor: np.ndarray, axis_in_parent: np.ndarray, target: Component | None) -> "Domino":
        obb = self.obb_in_world()
        anchor_in_world = self.to_world() * anchor
        parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
        axis_in_parent = axis_in_parent / np.linalg.norm(axis_in_parent)
        axis_in_world = parent_to_world.rotation @ axis_in_parent
        rotate_angle = None
        target_obbs = []
        if target is not None:
            target_obbs = [target_domino.obb_in_world() for target_domino in target.collect_dominoes()]
        else:
            target_obbs = [GROUND_OBB]
        for target_obb in target_obbs:
            target_rotate_angle = SAT.rotate_angle(obb, target_obb, anchor_in_world, axis_in_world)
            if target_rotate_angle is not None:
                if rotate_angle is None or target_rotate_angle < rotate_angle:
                    rotate_angle = target_rotate_angle
        if rotate_angle is not None:
            from_world = self.to_world().inverse()
            self.rotate(anchor, from_world.rotation @ axis_in_world, rotate_angle)
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

