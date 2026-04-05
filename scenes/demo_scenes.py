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

def scene_or_gate_showcase() -> Component:
    scene = Component()
    scene.add_child("gate", (
        OrGate()
    ))
    return scene

def scene_or_gate_run() -> Component:
    scene = Component()
    scene.add_child("gate", (
        OrGate()
    ))
    scene.add_child("start_1", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, 0.2, 0])))
    ))
    scene.add_child("start_2", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, -0.2, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, 0, 0])))
    ))
    scene.connect("start_1", "out", "gate", "in_1")
    scene.connect("start_2", "out", "gate", "in_2")
    scene.connect("gate", "out", "end", "in")
    return scene

def scene_and_gate_showcase() -> Component:
    scene = Component()
    scene.add_child("gate", (
        LeanAndGate()
    ))
    return scene

def scene_and_gate_run() -> Component:
    scene = Component()
    scene.add_child("gate", (
        LeanAndGate()
    ))
    scene.add_child("start_1", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, 0.2, 0])))
    ))
    scene.add_child("start_2", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, -0.2, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, 0, 0])))
    ))
    scene.connect("start_1", "out", "gate", "in_1")
    scene.connect("start_2", "out", "gate", "in_2")
    scene.connect("gate", "out", "end", "in")
    return scene

def scene_condition_gate_showcase() -> Component:
    scene = Component()
    scene.add_child("gate", (
        ConditionGate()
    ))
    return scene

def scene_condition_gate_run() -> Component:
    scene = Component()
    scene.add_child("gate", (
        ConditionGate()
    ))
    scene.add_child("start", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, 0, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, 0, 0])))
    ))
    scene.connect("start", "out", "gate", "in")
    scene.connect("gate", "out", "end", "in")
    scene.add_child("condition", (
        Domino().standing(-np.pi / 2)
        .place("z-", scene.anchor(np.array([0, 1, 0])))
    ))
    scene.connect("condition", "out", "gate", "condition")
    return scene

def scene_negative_condition_gate_showcase() -> Component:
    scene = Component()
    scene.add_child("gate", (
        NegativeConditionGate()
    ))
    return scene

def scene_negative_condition_gate_run() -> Component:
    scene = Component()
    scene.add_child("gate", (
        NegativeConditionGate()
    ))
    scene.add_child("start", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, 0, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, 0, 0])))
    ))
    scene.add_child("condition", (
        Domino().standing(-np.pi / 2)
        .place("z-", scene.anchor(np.array([-0.05, 1, 0])))
    ))
    scene.connect("start", "out", "gate", "in")
    scene.connect("gate", "out", "end", "in")
    scene.connect("condition", "out", "gate", "condition")
    return scene
