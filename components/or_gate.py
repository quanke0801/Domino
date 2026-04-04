import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino


class OrGate(Component):
    def __init__(self):
        super().__init__()
        self.add_child("out", (
            Domino.standing()
            .place("x-z-", self.anchor(""))
        ))
        self.add_child("in_1", (
            Domino.standing()
            .place("x+y-", self.child("out").anchor("x-y+"))
            .move(np.array([0, -Domino.SIZE[0], 0]))
        ))
        self.add_child("in_2", (
            Domino.standing()
            .place("x+y+", self.child("out").anchor("x-y-"))
            .move(np.array([0, Domino.SIZE[0], 0]))
        ))
        self.add_socket("in_1", "in_1")
        self.add_socket("in_2", "in_2")
        self.add_socket("out", "out")
