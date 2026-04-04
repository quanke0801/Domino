import numpy as np

from Domino.components.component import Component, PointRef, VectorRef
from Domino.components.domino import Domino


class PileDomino(Component):
    def __init__(self, pile_count: int):
        super().__init__()
        if pile_count <= 0:
            raise ValueError(f"pile_count must be positive, got {pile_count}")
        self.pile_count = pile_count
        self.size = np.array([Domino.SIZE[2], Domino.SIZE[1], Domino.SIZE[0] * pile_count])
        self.add_child("0", (
            Domino().lying()
            .place("x+", self.anchor(""))
        ))
        for i in range(1, pile_count):
            self.add_child(f"{i}", (
                Domino().lying()
                .place("x+", self.child(f"{i - 1}").anchor("x-"))
            ))
        for sign_x, label_x in [(-1, "x-"), (0, ""), (1, "x+")]:
            for sign_y, label_y in [(-1, "y-"), (0, ""), (1, "y+")]:
                # NOTE PileDomino's origin is at bottom center.
                for sign_z, label_z in [(0, "z-"), (1, ""), (2, "z+")]:
                    anchor_name = f"{label_x}{label_y}{label_z}"
                    anchor_position_local = np.array([sign_x, sign_y, sign_z]) * self.size / 2
                    self.anchors[anchor_name] = anchor_position_local
