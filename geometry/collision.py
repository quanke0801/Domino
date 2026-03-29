import numpy as np

from Domino.geometry.pose import Pose, rotation_matrix_from_axis_angle

EPSILON = 1e-8


class OBB:
    def __init__(self, pose: Pose, half_extents: np.ndarray):
        self.pose = pose
        self.half_extents = half_extents
    
    def axes(self) -> list[np.ndarray]:
        return [self.pose.rotation[:, i] for i in range(3)]
    
    def corners(self) -> np.ndarray:
        corners = []
        for sign_x in [-1, 1]:
            for sign_y in [-1, 1]:
                for sign_z in [-1, 1]:
                    sign_vector = np.array([sign_x, sign_y, sign_z])
                    corner = self.pose * (sign_vector * self.half_extents)
                    corners.append(corner)
        return np.array(corners)
    
    def radius_along_axis(self, axis: np.ndarray) -> float:
        return np.sum(np.abs(self.pose.rotation.T @ axis) * self.half_extents)
    
    def rotate(self, anchor: np.ndarray, axis: np.ndarray, angle: float) -> "OBB":
        translation = Pose(position = anchor)
        rotation = Pose(rotation = rotation_matrix_from_axis_angle(axis, angle))
        return OBB(translation * rotation * translation.inverse() * self.pose, self.half_extents)


class SAT:
    def __init__(self):
        pass
    
    @staticmethod
    def generate_separation_axes(obb1: OBB, obb2: OBB) -> list[np.ndarray]:
        # obb1 axes, obb2 axes, and cross products of obb1 and obb2 axes
        separation_axes = obb1.axes() + obb2.axes()
        for axis1 in obb1.axes():
            for axis2 in obb2.axes():
                axis = np.cross(axis1, axis2)
                axis_length = np.linalg.norm(axis)
                if axis_length < EPSILON:
                    continue
                separation_axes.append(axis / axis_length)
        return separation_axes
    
    @staticmethod
    def slide_t(obb1: OBB, obb2: OBB, velocity1: np.ndarray) -> float | None:
        # For each separation axis, compute the overlap interval.
        t_first, t_last = -np.inf, np.inf
        for separation_axis in SAT.generate_separation_axes(obb1, obb2):
            # Compute initial properties: sum of radii, initial distance, and relative velocity.
            sum_radius = obb1.radius_along_axis(separation_axis) + obb2.radius_along_axis(separation_axis)
            initial_distance = np.dot(obb2.pose.position - obb1.pose.position, separation_axis)
            relative_velocity = -np.dot(velocity1, separation_axis)
            # Determine overlapping interval.
            if np.abs(relative_velocity) < EPSILON:
                if np.abs(initial_distance) > sum_radius - EPSILON: # to avoid tangential slide
                    return None # Never will collide along this axis, can return early.
                else:
                    pass # No constraint on t_first/t_last along this axis.
            else:
                # math: |d0 + v * t| <= r
                t1 = (-sum_radius - initial_distance) / relative_velocity
                t2 = (sum_radius - initial_distance) / relative_velocity
                t_enter, t_exit = min(t1, t2), max(t1, t2)
                t_first, t_last = max(t_first, t_enter), min(t_last, t_exit)
                if t_first > t_last + EPSILON:
                    return None # No overlap interval, can return early.
        return t_first
    
    @staticmethod
    def separation_distance(obb1: OBB, obb2: OBB) -> float | None:
        distance = -np.inf
        for separation_axis in SAT.generate_separation_axes(obb1, obb2):
            sum_radius = obb1.radius_along_axis(separation_axis) + obb2.radius_along_axis(separation_axis)
            initial_distance = np.dot(obb2.pose.position - obb1.pose.position, separation_axis)
            distance = max(distance, initial_distance - sum_radius)
        return distance
    
    @staticmethod
    def rotate_angle(obb1: OBB, obb2: OBB, anchor1: np.ndarray, axis1: np.ndarray, tolerance: float = 1.0E-4) -> float | None:
        axis1 = axis1 / np.linalg.norm(axis1)
        # Find max corner radius from obb1.
        max_corner_radius = np.max(np.linalg.norm(np.cross(axis1, obb1.corners() - anchor1), axis=1))
        # Conservative advancement loop.
        accumulated_angle = 0.0
        rotated_obb1 = obb1
        while accumulated_angle < 2 * np.pi:
            safe_distance = SAT.separation_distance(rotated_obb1, obb2)
            if safe_distance < 0:
                return None if accumulated_angle == 0 else accumulated_angle
            else:
                safe_angle = safe_distance / max_corner_radius
                accumulated_angle += safe_angle
                rotated_obb1 = obb1.rotate(anchor1, axis1, accumulated_angle)
                if safe_distance < tolerance:
                    return accumulated_angle
        return None

if __name__ == "__main__":
    obb1 = OBB(Pose(np.array([0, 0, 0]), np.eye(3)), np.array([1, 1, 1]))
    obb2 = OBB(Pose(np.array([2.2, 0, 0]), np.eye(3)), np.array([1, 1, 1]))
    angle = SAT.rotate_angle(obb1, obb2, np.array([0, 0, 0]), np.array([0, 0, 1]))
    print(angle)
    rotated_obb1 = obb1.rotate(np.array([0, 0, 0]), np.array([0, 0, 1]), angle)
    print(rotated_obb1.corners())
