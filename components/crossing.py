import numpy as np

from Domino.components.component import Component
from Domino.components.domino import Domino
from Domino.components.impulse_trigger import ImpulseTrigger


class Crossing(Component):
    def __init__(self):
        super().__init__()
        self.add_child("base_shaft", (
            Domino.lying()
            .place("x+", self.anchor(""))
        ))
        self.add_child("impulse_trigger_1", (
            ImpulseTrigger()
            .rotate(self.anchor(""), self.axis("z+"), np.pi)
            .place("", self.anchor(np.array([-1, 0, 0])))
            .move_to_touch(self.axis("x+"), self.child("base_shaft"))
        ))
        self.add_child("impulse_trigger_2", (
            ImpulseTrigger()
            .place("", self.anchor(np.array([1, 0, 0])))
            .move_to_touch(self.axis("x-"), self.child("base_shaft"))
        ))
        self.add_child("overpass", (
            Domino.standing(np.pi / 2)
            .place("z-", self.child("base_shaft").anchor("x-"))
        ))
        self.add_socket("in_1", self.child("impulse_trigger_1").socket("in"))
        self.add_socket("out_1", self.child("impulse_trigger_2").socket("out"))
        self.add_socket("in_1_reversed", self.child("impulse_trigger_2").socket("in"))
        self.add_socket("out_1_reversed", self.child("impulse_trigger_1").socket("out"))
        self.add_socket("in_2", self.child("overpass").socket("in"))
        self.add_socket("out_2", self.child("overpass").socket("out"))
        self.add_socket("in_2_reversed", self.child("overpass").socket("in_reversed"))
        self.add_socket("out_2_reversed", self.child("overpass").socket("out_reversed"))
