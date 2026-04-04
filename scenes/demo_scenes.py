import numpy as np

from Domino.components import *

def scene_line_domino_analogy() -> Component:
    scene = Component()
    # scene.add_child("fishbone_time_delay", (
    #     LineDomino(
    #         PointRef(Ground(), np.array([-1, 0, 0])),
    #         PointRef(Ground(), np.array([1, 0, 0])),
    #         gap_ratio=0.9,
    #         fishbone_angle=np.pi / 6
    #     )
    # ))
    scene.add_child("line_domino", (    
        LineDomino(
            PointRef(Ground(), np.array([-1, 0, 0])),
            PointRef(Ground(), np.array([1, 0, 0])),
            # include=(True, False),
        ).trigger()
    ))
    return scene
