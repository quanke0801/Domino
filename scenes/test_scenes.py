import numpy as np

from Domino.components import *

def build_scene_2() -> Component:
    scene = Component()
    scene.add_child("support", (
        Domino().lying(np.pi / 2)
        .place("x+", scene.anchor(""))
    ))
    scene.add_child("output", (
        Domino().standing()
        .place("x+z-", scene.child("support").anchor("x-y-"))
        .move(np.array([0.005, 0, 0]))
    ))
    scene.add_child("shaft_slanted", (
        Domino().lying()
        .place("x+z+", scene.child("output").anchor("x-z-"))
        # .move(np.array([0.0009, 0, 0]))
        .rotate_to_touch(np.array([0.0075, 0, 0.015]), np.array([0, -1, 0]), Ground())
    ))
    # scene.add_child("shaft_flat", (
    #     Domino().lying()
    #     .place("x+", scene.anchor(np.array([-1, 0, 0])))
    #     .move_to_touch(np.array([1, 0, 0]), scene.child("shaft_slanted"))
    # ))
    # scene.add_child("shim", (
    #     Domino()
    #     .orient(Domino.sideways())
    #     .place_snap("x+y-", scene.child("shaft_flat").anchor("x+z-"))
    #     .move(np.array([-0.01, 0, 0]))
    # ))
    # scene.add_child("trigger", (
    #     Domino().standing()
    #     .place("x-z-", scene.child("support").anchor("x+y+"))
    #     .move(np.array([0.125, 0, 0]))
    #     .rotate("x+z-", np.array([0, 1, 0]), np.radians(-10))
    # ))
    return scene

def build_scene_3() -> Component:
    scene = Component()
    # scene.add_child("x_axis", (
    #     Domino.lying()
    #     .place("x+", scene.anchor(""))
    # ))
    # scene.add_child("y_axis", (
    #     Domino.lying(np.pi / 2)
    #     .place("x+", scene.child("x_axis").anchor("x-"))
    # ))
    scene.add_child("line", (
        LineDomino(
            scene.anchor(np.array([-2, 0, 0])),
            scene.anchor(np.array([2, 0, 0]))
        ).trigger()
    ))
    # trigger = scene.child("line").child("0")
    # trigger.rotate("x+z-", trigger.axis("y+"), np.radians(10))

    return scene

def build_scene_4() -> Component:
    scene = Component()
    scene.add_child("gate", (
        ConditionGate()
    ))
    y = scene.child("gate").child("in").to_world().position[1]
    scene.add_child("start", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, y, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, y, 0])))
    ))
    scene.add_child("condition", (
        Domino().standing(-np.pi / 2)
        .place("z-", scene.anchor(np.array([0, 1, 0])))
    ))
    scene.connect("start", "out", "gate", "in")
    scene.connect("gate", "out", "end", "in")
    scene.connect("condition", "out", "gate", "condition")
    return scene

def build_scene_5() -> Component:
    scene = Component()
    scene.add_child("gate", (
        ElegantAndGate()
    ))
    scene.add_child("in_line_1", (
        LineDomino.to_domino(scene.child("gate").child("in_1"), 1)
    ))
    scene.add_child("in_line_2", (
        LineDomino.to_domino(scene.child("gate").child("in_2"), 1)
    ))
    scene.add_child("out_line", (
        LineDomino.from_domino(scene.child("gate").child("out"), 1)
    ))
    return scene

def build_scene_6() -> Component:
    scene = Component()
    scene.add_child("gate", (
        LeanAndGate()
    ))
    scene.add_child("start_1", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, 0.1, 0])))
    ))
    scene.add_child("start_2", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([-1, -0.1, 0])))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([1, 0, 0])))
    ))
    scene.connect("start_1", "out", "gate", "in_1")
    scene.connect("start_2", "out", "gate", "in_2")
    scene.connect("gate", "out", "end", "in")
    return scene

def build_scene_7() -> Component:
    scene = Component()
    scene.add_child("gate", (
        OrGate()
    ))
    scene.add_child("in_line_1", (
        LineDomino.to_domino(scene.child("gate").child("in_1"), 1)
    ))
    scene.add_child("in_line_2", (
        LineDomino.to_domino(scene.child("gate").child("in_2"), 1)
    ))
    scene.add_child("out_line", (
        LineDomino.from_domino(scene.child("gate").child("out"), 1)
    ))
    return scene

def build_scene_8() -> Component:
    scene = Component()
    scene.add_child("start", (
        Domino().standing()
        .place("z-", scene.anchor(""))
    ))
    scene.add_child("end", (
        Domino().standing()
        .place("z-", scene.anchor(np.array([2, 0.3, 0])))
        .rotate("", "z+", np.pi / 2)
    ))
    scene.add_socket("start", "start")
    scene.add_socket("end", "end")
    # scene.add_child("curve", (
    #     CurveDomino(
    #         scene.socket("start"),
    #         scene.socket("end")
    #     )
    # ))
    scene.connect("start", "out", "end", "in")
    return scene

def test_scene_copy() -> Component:
    scene = Component()
    scene.add_child("gate", (
        LeanAndGate()
    ))
    scene.add_child("other_gate", (
        scene.child("gate").copy()
        .mirror(
            scene.anchor(np.array([0.3, 0, 0])),
            scene.axis(np.array([0.8, 0.6, 0]))
        )
    ))
    return scene
