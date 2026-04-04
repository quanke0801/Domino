import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.pile_domino import PileDomino
from Domino.components.ground import Ground

class ConditionGate(Component):
    def __init__(self):
        super().__init__()
        self.add_child("base", (
            PileDomino(5)
            .place("z-", self.anchor(""))
            .rotate("", np.array([0, 0, 1]), np.pi / 2)
        ))
        self.add_child("blocker", (
            PileDomino(3)
            .rotate("x+z-", "y+", np.pi / 2)
            .place("x+y+", self.child("base").anchor("x-z-"))
        ))
        # TODO Might be better to use sideways connection.
        self.add_child("connection", (
            Domino.lying()
            .place("x+y+", self.child("base").anchor("x+z+"))
            .move(self.axis(np.array([0, Domino.SIZE[0], 0])))
        ))
        self.add_child("pusher", (
            Domino.standing(-np.pi / 2)
            .place("x+z-", self.child("connection").anchor("y+"))
            .move_to_touch(self.axis(np.array([0, 0, -1])), Ground())
        ))
        # self.add_child("connection", (
        #     Domino.sideways(np.pi / 2)
        #     .place("x+y-", self.child("base").anchor("x+z+"))
        # ))
        # self.add_child("pusher", (
        #     Domino.standing(-np.pi / 2)
        #     .place("x+z-", self.child("base").anchor("x+z-"))
        #     .move(np.array([-0.01, 0, 0]))
        # ))
        self.add_child("temp", (
            Domino.lying()
            .place("x+y-", self.child("base").anchor("x-z-"))
        ))
        self.add_child("in", (
            Domino.standing()
            .place("x+z-", self.child("temp").anchor("x+z-"))
            .move(np.array([-Domino.SIZE[0], 0, 0]))
        ))
        self.add_child("out", (
            Domino.standing()
            .place("x-z-", self.child("temp").anchor("x+z+"))
            .move(np.array([Domino.SIZE[0], 0, 0]))
        ))
        del self.children["temp"]

        self.add_socket("in", "in")
        self.add_socket("out", "out")
        self.add_socket("condition", "pusher")
