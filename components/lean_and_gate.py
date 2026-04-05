import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.pile_domino import PileDomino


class LeanAndGate(Component):
    def __init__(self):
        super().__init__()
        half_width = Domino.SIZE[1] / 2
        split = Domino.SIZE[0] + Domino.SIZE[1] / 2
        self.add_child("support_1", (
            Domino.standing()
            .place("y-z-", self.anchor(""))
            .move(np.array([-half_width, split, 0]))
        ))
        self.add_child("blocker_1", (
            Domino.standing(np.pi / 2)
            .place("x-z-", self.anchor(""))
            .rotate_to_touch("x+z-", np.array([0, 1, 0]), self.child("support_1"))
        ))
        self.add_child("support_2", (
            Domino.standing()
            .place("y+z-", self.anchor(""))
            .move(np.array([-half_width, -split, 0]))
        ))
        self.add_child("blocker_2", (
            Domino.standing(-np.pi / 2)
            .place("x-z-", self.anchor(""))
            .rotate_to_touch("x+z-", np.array([0, 1, 0]), self.child("support_2"))
        ))
        self.add_child("trigger_base", (
            Domino.lying()
            .place("x+z+", self.anchor(np.array([-1, 0, 0])))
            .move_to_touch("z+", self.child("blocker_1"))
        ))
        self.add_child("trigger", (
            Domino.standing()
            .place("x+z-", self.anchor(""))
            .move(np.array([-split - Domino.SIZE[0], 0, 1]))
            .move_to_touch("z-", self.child("trigger_base"))
            .rotate_to_touch("x+z-", np.array([0, 1, 0]), self.child("blocker_1"))
        ))
        self.add_child("trigger_support", (
            Domino.lying()
            .place("x+z+", self.child("trigger_base").anchor("x-z-"))
            .move_to_touch("z+", self.child("trigger"))
        ))
        self.add_child("out", (
            Domino.standing()
            .place("z-", self.anchor(""))
            .move(np.array([1, 0, 0]))
            .move_to_touch("x-", self.child("blocker_1"))
        ))
        self.add_socket("in_1", "support_1")
        self.add_socket("in_2", "support_2")
        self.add_socket("out", "out")
