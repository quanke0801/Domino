import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.ground import Ground


class ImpulseTrigger(Component):
    EDGE_HANG = Domino.SIZE[0] * 0.4
    SHRINK_GAP_ITERATIONS = 3

    def __init__(self):
        super().__init__()
        self.add_child("base", (
            Domino.lying(np.pi / 2)
            .place("x+", self.anchor(np.array([Domino.SIZE[2] * 2, 0, 0])))
        ))
        self.add_child("shim", (
            Domino.sideways()
            .place("x+y-", self.child("base").anchor("x-y-"))
            .move(self.axis(np.array([ImpulseTrigger.EDGE_HANG, 0, 0])))
        ))
        self.add_child("shaft", (
            Domino.lying()
            .place("x+z+", self.child("shim").anchor("x-y-"))
            .rotate_to_touch(self.child("base").anchor("x-y+"), self.axis("y-"), Ground())
        ))
        for _ in range(ImpulseTrigger.SHRINK_GAP_ITERATIONS):
            self.child("shaft").move_to_touch("z+", self.child("shim"))
            self.child("shaft").rotate_to_touch(self.child("base").anchor("x-y+"), self.axis("y-"), Ground())
        self.add_child("temp", (
            Domino.lying()
            .place("x+z+", self.anchor(""))
        ))
        self.child("shaft").move_to_touch(self.axis("x-"), self.child("temp"))
        self.child("base").move_to_touch(self.axis("x-"), self.child("shaft"))
        self.child("shim").move_to_touch(self.axis("x-"), self.child("shaft"))
        self.add_child("trigger", (
            Domino.standing()
            .place("x-z-", self.child("shim").anchor("x+"))
            .move_to_touch(self.axis("z-"), Ground())
            .move(self.axis(np.array([Domino.SIZE[0], 0, 0])))
        ))
        del self.children["temp"]
        self.add_socket("in", self.child("trigger").socket("in_reversed"))
        self.add_socket("out", "trigger")
