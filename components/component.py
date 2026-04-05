from copy import deepcopy
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


@dataclass
class SocketRef:
    component: "Component | None"
    position: np.ndarray
    direction: np.ndarray

    def to_world(self) -> np.ndarray:
        if self.component is None:
            return self.position, self.direction
        component_to_world = self.component.to_world()
        return component_to_world.apply_point(self.position), component_to_world.apply_vector(self.direction)
    
    def to_other(self, other: "Component | None") -> np.ndarray:
        component_to_world, other_to_world = Pose(), Pose()
        if self.component is not None:
            component_to_world = self.component.to_world()
        if other is not None:
            other_to_world = other.to_world()
        component_to_other = other_to_world.inverse() * component_to_world
        return component_to_other.apply_point(self.position), component_to_other.apply_vector(self.direction)


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
    
    def add_child(self, name: str, child: "Component") -> None:
        child.parent = self
        self.children[name] = child
    
    def connect(self, child1: str, socket1: str, child2: str, socket2: str) -> None:
        from Domino.components.curve_domino import CurveDomino
        name = f"{child1}_{socket1}_to_{child2}_{socket2}"
        self.add_child(name, (
            CurveDomino(
                self.child(child1).socket(socket1),
                self.child(child2).socket(socket2)
            )
        ))

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
    
    def socket(self, name: str) -> SocketRef:
        return SocketRef(self, self.sockets[name][0], self.sockets[name][1])
    
    def add_socket(self, socket_name: str, socket: "str | SocketRef | Domino") -> None:
        from Domino.components.domino import Domino
        if isinstance(socket, str):
            position = self.child(socket).anchor("").to_other(self)
            direction = self.child(socket).axis("x+").to_other(self)
        elif isinstance(socket, SocketRef):
            position, direction = socket.to_other(self)
        elif isinstance(socket, Domino):
            position = socket.anchor("").to_other(self)
            direction = socket.axis("x+").to_other(self)
        else:
            raise ValueError(f"unsupported socket type: {type(socket)}")
        self.sockets[socket_name] = (position, direction)

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
    
    def mirror(self, origin: str | np.ndarray | PointRef, normal: str | np.ndarray | VectorRef) -> "Component":
        # Resolve origin in parent frame.
        if isinstance(origin, str):
            origin_local = self.anchors[origin]
        elif isinstance(origin, np.ndarray):
            origin_local = origin
        elif isinstance(origin, PointRef):
            origin_local = origin.to_other(self)
        else:
            raise ValueError(f"unsupported origin type: {type(origin)}")
        # Resolve normal in parent frame.
        if isinstance(normal, str):
            normal_local = Component.NAME_TO_AXIS[normal]
        elif isinstance(normal, np.ndarray):
            normal_local = normal
        elif isinstance(normal, VectorRef):
            normal_local = normal.to_other(self)
        else:
            raise ValueError(f"unsupported normal type: {type(normal)}")
        origin_in_parent = self.to_parent.apply_point(origin_local)
        normal_in_parent = self.to_parent.apply_vector(normal_local)
        normal_in_parent /= np.linalg.norm(normal_in_parent)
        normal_projection = 2 * np.outer(normal_in_parent, normal_in_parent)
        plane_reflection = np.eye(3) - normal_projection
        mirror = Pose(normal_projection @ origin_in_parent, plane_reflection)
        self.to_parent = mirror * self.to_parent
        return self
    
    def move_to_touch(self, direction: str | np.ndarray | VectorRef, target: "Component | list[Component]") -> "Component":
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
        contact_distance = None
        for leaf in self.collect_leaves():
            leaf_obb = leaf.obb_in_world()
            for target_obb in target_obbs:
                target_contact_distance = SAT.slide_distance(leaf_obb, target_obb, direction_in_world)
                if target_contact_distance is not None and target_contact_distance > 0:
                    if contact_distance is None or target_contact_distance < contact_distance:
                        contact_distance = target_contact_distance
        if contact_distance is not None:
            delta_in_world = contact_distance * direction_in_world
            delta_local = self.to_world().inverse().apply_vector(delta_in_world)
            self.move(delta_local)
        else:
            logger.warning(f"No contact found or already colliding with target {target}.")
        return self
    
    def rotate_to_touch(self, anchor: None | str | np.ndarray | PointRef, axis: str | np.ndarray | VectorRef, target: "Component | list[Component]") -> "Component":
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
        rotate_angle = None
        for leaf in self.collect_leaves():
            leaf_obb = leaf.obb_in_world()
            for target_obb in target_obbs:
                target_rotate_angle = SAT.rotate_angle(leaf_obb, target_obb, anchor_in_world, axis_in_world)
                if target_rotate_angle is not None:
                    if rotate_angle is None or target_rotate_angle < rotate_angle:
                        rotate_angle = target_rotate_angle
        if rotate_angle is not None:
            self.rotate(anchor, axis, rotate_angle)
        else:
            logger.warning(f"No contact found or already colliding with target {target}.")
        return self

    def collect_leaves(self) -> list["Component"]:
        leaves = []
        if len(self.children) == 0:
            leaves.append(self)
        else:
            for child in self.children.values():
                leaves.extend(child.collect_leaves())
        return leaves
