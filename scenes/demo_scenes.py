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

def scene_half_adder() -> Component:
    scene = Component()
    scale = 0.5
    scene.add_child("A", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-scale * 3, scale, 0])))
    ))
    scene.add_child("A_branch", (
        SideBranch()
        .place("", scene.anchor(np.array([-scale * 2, scale, 0])))
        .mirror("", "y+")
    ))
    scene.connect("A", "out", "A_branch", "in")
    scene.add_child("B", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-scale * 3, -scale, 0])))
    ))
    scene.add_child("B_branch", (
        SideBranch()
        .place("", scene.anchor(np.array([-scale * 2, -scale, 0])))
    ))
    scene.connect("B", "out", "B_branch", "in")
    scene.add_child("crossing", (
        Crossing()
        .place("", scene.anchor(np.array([-scale, 0, 0])))
        .rotate("", "z+", np.pi / 4)
    ))
    scene.connect("A_branch", "out_2", "crossing", "in_2_reversed")
    scene.connect("B_branch", "out_2", "crossing", "in_1")
    scene.add_child("or", (
        OrGate()
        .place("", scene.anchor(np.array([0, scale, 0])))
    ))
    scene.add_child("and", (
        LeanAndGate()
        .place("", scene.anchor(np.array([scale * 0.5, -scale, 0])))
    ))
    scene.add_child("pre_and", (
        LineDomino.to_socket(scene.child("and").socket("in_1"), scale * 0.5)
    ))
    scene.connect("A_branch", "out_1", "or", "in_1")
    scene.connect("B_branch", "out_1", "and", "in_2")
    scene.connect("crossing", "out_1", "or", "in_2")
    scene.connect("crossing", "out_2_reversed", "pre_and", "in")
    scene.add_child("and_branch", (
        SideBranch()
        .place("", scene.anchor(np.array([scale * 1.5, -scale, 0])))
    ))
    scene.connect("and", "out", "and_branch", "in")
    # scene.add_child("delay",
    #     LineDomino(
    #         PointRef(scene, np.array([scale * 0.2, scale, 0])),
    #         PointRef(scene, np.array([scale * 1.8, scale, 0])),
    #         (True, True),
    #         0.8,
    #         np.pi / 6,
    #     )
    # )
    scene.add_child("or_branch", (
        SideBranch()
        .place("", scene.anchor(np.array([scale * 0.2, scale, 0])))
        .mirror("", "y+")
    ))
    scene.connect("or", "out", "or_branch", "in")
    scene.add_child("uturn_1", (
        UTurn()
        .place("", scene.anchor(np.array([scale * 0.6, -scale * 0.25, 0])))
        .rotate("", "z+", -np.pi / 2)
    ))
    scene.connect("or_branch", "out_2", "uturn_1", "in_1")
    scene.add_child("uturn_2", (
        UTurn()
        .place("", scene.anchor(np.array([scale * 0.9, scale, 0])))
        .rotate("", "z+", np.pi / 2)
    ))
    scene.connect("uturn_1", "out_1", "uturn_2", "in_2")
    scene.add_child("uturn_3", (
        UTurn()
        .place("", scene.anchor(np.array([scale * 1.2, -scale * 0.25, 0])))
        .rotate("", "z+", -np.pi / 2)
    ))
    scene.connect("uturn_2", "out_2", "uturn_3", "in_1")
    scene.add_child("uturn_branch", (
        SideBranch()
        .place("", scene.anchor(np.array([scale * 1.4, scale, 0])))
        .rotate("", "z+", np.pi / 2)
        .mirror("", "y+")
    ))
    scene.connect("uturn_3", "out_1", "uturn_branch", "in")
    scene.add_child("not", (
        NegativeConditionGate()
        .place("", scene.anchor(np.array([scale * 2.5, scale, 0])))
        .mirror("", "y+")
    ))
    scene.connect("uturn_branch", "out_2", "not", "in")
    scene.connect("and_branch", "out_2", "not", "condition")
    scene.add_child("S", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([scale * 3, scale, 0])))
    ))
    scene.add_child("C", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([scale * 3, -scale, 0])))
    ))
    
    scene.connect("not", "out", "S", "in")
    scene.connect("and_branch", "out_1", "C", "in")

    scene.child("A").trigger()
    scene.child("B").trigger()

    return scene
