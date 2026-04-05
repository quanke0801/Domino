import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.pile_domino import PileDomino


class NegativeConditionGate(Component):
    def __init__(self):
        super().__init__()
        self.add_child("pile", (
            PileDomino(6)
            .place("z-", self.anchor(""))
            .rotate("", "z+", np.pi / 2)
        ))
        self.add_child("stablizer_1", (
            Domino.standing(np.pi / 2)
            .place("x+y+z-", self.child("pile").anchor("x+y-z-"))
        ))
        self.add_child("stablizer_2", (
            Domino.standing(np.pi / 2)
            .place("x-y+z-", self.child("pile").anchor("x-y-z-"))
        ))
        self.add_child("temp_1", (
            Domino.standing()
            .place("x+z-", self.child("pile").anchor("y+z-"))
        ))
        self.add_child("pillar", (
            Domino.standing(-np.pi / 2)
            .place("y+", self.child("temp_1").anchor("x-"))
        ))
        self.add_child("temp_2", (
            Domino.standing()
            .place("x+", self.child("pillar").anchor("y-"))
        ))
        self.add_child("shaft", (
            Domino.lying()
            # .place("x+", self.child("pillar").anchor("z+"))
            .place("x+z-", self.child("temp_2").anchor("x-z+"))
        ))
        self.add_child("support", (
            Domino.lying(np.pi / 2)
            .place("x+y-", self.child("temp_2").anchor("x-z-"))
        ))
        self.add_child("in", (
            Domino.standing()
            .place("z-", self.child("support").anchor("x-"))
        ))
        self.add_child("temp_3", (
            Domino.standing()
            .place("x-z-", self.child("pile").anchor("y-z-"))
        ))
        self.add_child("out", (
            Domino.standing()
            .place("x-", self.child("temp_3").anchor("x+"))
        ))
        del self.children["temp_1"]
        del self.children["temp_2"]
        del self.children["temp_3"]
        self.add_socket("in", "in")
        self.add_socket("out", "out")
        self.add_socket("condition", "pillar")

