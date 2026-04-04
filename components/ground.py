import numpy as np

from Domino.components.component import Component
from Domino.geometry.pose import Pose
from Domino.geometry.collision import OBB


class Ground(Component):
    _instance = None
    HALF_SIZE = np.array([10, 10, 1])
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def to_world(self) -> Pose:
        return Pose(position=np.array([0, 0, -Ground.HALF_SIZE[2]]))
    
    def obb_in_world(self) -> OBB:
        return OBB(self.to_world(), self.HALF_SIZE)
    
    def collect_leaves(self) -> list["Component"]:
        return [self]
