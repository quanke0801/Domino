import numpy as np

from Domino.components.component import Component, PointRef, SocketRef
from Domino.components.domino import Domino
from Domino.components.ground import Ground


class UTurn(Component):
    def __init__(self):
        super().__init__()
        self.add_child("base", (
            Domino().sideways(np.pi / 2)
            .place("y-", self.anchor(""))
        ))
        self.add_child("lever", (
            Domino().sideways()
            .place("y-", self.child("base").anchor("y+"))
        ))
        self.add_child("pivot", (
            Domino().standing(np.pi / 2)
            .place("y+z-", self.child("lever").anchor("x+y-"))
        ))
        self.add_child("left", (
            Domino().standing()
            .place("x+z-", self.child("lever").anchor("x-z-"))
            .move_to_touch("z-", Ground())
        ))
        self.add_child("right", (
            Domino().standing()
            .place("x+z-", self.child("lever").anchor("x-z+"))
            .move_to_touch("z-", Ground())
        ))
        self.add_socket("in_1", "right")
        self.add_socket("out_1", self.child("left").socket("out_reversed"))
        self.add_socket("in_2", "left")
        self.add_socket("out_2", self.child("right").socket("out_reversed"))
