import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino

class ElegantAndGate(Component):
    def __init__(self):
        super().__init__()
        self.add_child("trap", (
            Domino.standing(np.pi / 2)
            .place("z-", self.anchor(""))
        ))
        angle = 0
        overlap = Domino.SIZE[0] / 6
        self.add_child("fill_1", (
            Domino.standing()
            .place("x+y-", self.child("trap").anchor("x+y+"))
            .rotate("x+y-", "z+", -angle)
            .move(np.array([0, -overlap, 0]))
        ))
        self.add_child("in_1", (
            Domino.standing()
            .place("x+y-", self.child("fill_1").anchor("x-y+"))
            .move(np.array([-Domino.SIZE[2], -overlap, 0]))
            .move_to_touch("x+", self.child("fill_1"))
        ))
        self.add_child("fill_2", (
            Domino.standing()
            .place("x+y+", self.child("trap").anchor("x-y+"))
            .rotate("x+y+", "z+", angle)
            .move(np.array([0, overlap, 0]))
        ))
        self.add_child("in_2", (
            Domino.standing()
            .place("x+y+", self.child("fill_2").anchor("x-y-"))
            .move(np.array([-Domino.SIZE[2], overlap, 0]))
            .move_to_touch("x+", self.child("fill_2"))
        ))
        self.add_child("out_temp", (
            Domino.standing()
            .place("x-", self.child("trap").anchor("y-"))
            .move(np.array([Domino.SIZE[0], 0, 0]))
        ))
        self.add_child("out", (
            Domino.standing()
            .place("x-", self.child("out_temp").anchor("x+"))
        ))
        self.add_socket("in_1", "in_1")
        self.add_socket("in_2", "in_2")
        self.add_socket("out", "out")
