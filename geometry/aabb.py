import numpy as np

class AABB:
    def __init__(self, lower_bound: np.ndarray, upper_bound: np.ndarray):
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
