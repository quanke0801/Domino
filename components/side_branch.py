import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.ground import Ground


class SideBranch(Component):
    def __init__(self):
        super().__init__()
        self.add_child("in", (
            Domino.standing()
            .place("z-", self.anchor(""))
        ))
        self.add_child("lever", (
            Domino.sideways()
            .place("x-z+", self.child("in").anchor("x+"))
            .rotate("x-z+", self.axis("z+"), -np.pi / 4)
            .move(self.axis(np.array([0, 0, Domino.SIZE[2]])))
        ))
        self.add_child("base", (
            Domino.sideways(np.pi / 4)
            .place("", self.child("lever").anchor(""))
            .move_to_touch(self.axis("z-"), Ground())
        ))
        self.child("lever").move_to_touch(self.axis("z-"), self.child("base"))
        self.add_child("out_1", (
            Domino.standing()
            .place("z-", self.anchor(np.array([Domino.SIZE[2], 0, 0])))
            .move_to_touch("x-", self.child("base"))
        ))
        self.add_child("out_2", (
            Domino.standing(np.pi / 2)
            .place("x-z-", self.child("lever").anchor("x-z-"))
            .move_to_touch(self.axis("z-"), Ground())
        ))
        self.add_socket("in", "in")
        self.add_socket("out_1", "out_1")
        self.add_socket("out_2", "out_2")
