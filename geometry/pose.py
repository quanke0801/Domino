import numpy as np

def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    skew_axis = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * skew_axis + (1 - np.cos(angle)) * (skew_axis @ skew_axis)
    return R

def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return R_x @ R_y @ R_z


class Pose:
    def __init__(
            self,
            position: np.ndarray = np.array([0, 0, 0]),
            rotation: np.ndarray = np.eye(3)
    ):
        # TODO Check if position and rotation are valid.
        self.position = np.array(position)
        self.rotation = np.array(rotation)
    
    def set_position(self, position: np.ndarray):
        self.position = position
    
    def set_rotation(self, rotation: np.ndarray):
        self.rotation = rotation
    
    def __mul__(self, other: "Pose | np.ndarray"):
        if isinstance(other, Pose):
            return Pose(self.position + self.rotation @ other.position, self.rotation @ other.rotation)
        elif isinstance(other, np.ndarray):
            return self.position + self.rotation @ other
        raise TypeError(f"unsupported operand type: {type(other)}")

    def inverse(self) -> "Pose":
        return Pose(self.rotation.T @ -self.position, self.rotation.T)
