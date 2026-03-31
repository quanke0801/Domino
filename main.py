import time
import logging
logging.basicConfig(level=logging.INFO)

import numpy as np
import mujoco
import mujoco.viewer

from Domino.components.component import *
from Domino.geometry.pose import Pose

PALETTE = [
    "0.96 0.96 0.96", # White
    "0.92 0.39 0.12", # Orange
    "0.86 0.16 0.16", # Red
    # "0.55 0.24 0.71", # Purple
    # "0.90 0.35 0.55", # Pink
    # "0.39 0.20 0.16", # Brown
    "0.12 0.12 0.12", # Black
    "0.94 0.82 0.08", # Yellow
    "0.12 0.63 0.24", # Green
    "0.20 0.71 0.86", # Light Blue / Cyan
    "0.08 0.31 0.71"  # Dark Blue
]
SEED = 42
np.random.seed(SEED)
SLOW_DOWN_FACTOR = 2
TIME_STEP = 0.002

def CompileComponent(component: Component, to_world: Pose, xml_body_specs: list[str] = []) -> None:
    if type(component) == Domino:
        body_id = len(xml_body_specs)
        position_vector = to_world.position
        pos = f"{position_vector[0]} {position_vector[1]} {position_vector[2]}"
        rotation_matrix = to_world.rotation
        xyaxes = f"{rotation_matrix[0, 0]} {rotation_matrix[1, 0]} {rotation_matrix[2, 0]} {rotation_matrix[0, 1]} {rotation_matrix[1, 1]} {rotation_matrix[2, 1]}"
        xml_body_spec = f"""
        <body name="body_{body_id}" pos="{pos}" xyaxes="{xyaxes}">
            <freejoint/>
            <geom
                name="geom_{body_id}"
                type="box"
                size="{Domino.SIZE[0] / 2} {Domino.SIZE[1] / 2} {Domino.SIZE[2] / 2}"
                density="1000"
                friction="0.25 0.005 0.0001"
                rgba="{np.random.choice(PALETTE)} 1"/>
        </body>"""
        xml_body_specs.append(xml_body_spec)
        return
    for child_name, child in component.children.items():
        child_to_world = to_world * child.to_parent
        CompileComponent(child, child_to_world, xml_body_specs)

def CompileWorld(scene: Component) -> str:
    xml_body_specs = []
    CompileComponent(scene, scene.to_parent, xml_body_specs)
    ground_obb = Ground().obb_in_world()
    xml = f"""
<mujoco model="world">
  <option
    timestep="{TIME_STEP}"
    gravity="0 0 -9.81"
    integrator="implicit"
    cone="elliptic"
    solver="Newton"
    iterations="100"
    noslip_iterations="5"
    noslip_tolerance="0.00001"
  />
  <default>
    <geom solref="{TIME_STEP * 1} 1" solimp="0.99 0.99 0.002" friction="1 0.005 0.0001"/>
  </default>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.15 0.15 0.15"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2"/>
    <material name="grid" texture="grid" texrepeat="6 6" texuniform="true" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="-2 -2 5"/>
    <geom name="ground" type="plane" pos="0 0 0" size="{ground_obb.half_extents[0]} {ground_obb.half_extents[1]} 0.1" material="grid"
          friction="1.0 0.01 0.001" priority="1" />
{"\n".join(xml_body_specs)}
  </worldbody>
</mujoco>
"""
    return xml

def configure_camera(camera: mujoco.MjvCamera) -> None:
    camera.azimuth = 90
    camera.elevation = -20
    camera.distance = 1.5
    camera.lookat[:] = [0, 0, 0]

def build_scene_1() -> Component:
    scene = Component()
    scene.add_child("A", (
        Domino()
        .orient(Domino.standing(-np.pi / 8))
        .place_abs("z-", np.array([0, 0, 0]))
    ))
    scene.add_child("B", (
        Domino()
        .orient(Domino.standing(np.pi / 18))
        .rotate(None, np.array([0, 1, 0]), np.pi / 6)
        .place_abs("z-", np.array([0.2, 0, 0.04]))
        .move_to_touch(np.array([-1, 0, 0]), scene.children["A"])
        # .place_snap("z-", AnchorRef(scene.children["A"], "z+"))
    ))
    # scene.move(np.array([0.1, 0, 0.1]))
    # scene.rotate(AnchorRef(scene.children["A"], "x-y-z-"), np.array([0, 0, 1]), np.pi / 6)
    
    scene.add_child("P", (
        PileDomino(10)
        .move(np.array([-0.2, 0, 0.1]))
        .rotate(None, np.array([1, 1, 0]), np.pi / 3)
    ))

    scene.add_child("C", (
        Domino()
        .orient(Domino.sideways(-np.pi / 4))
        .rotate(None, np.array([0, 1, 0]), -np.pi / 6)
        .place_abs("", np.array([-0.2, -0.2, 0.1]))
        .move_to_touch(np.array([0.1, 1, -3.3]), None)
        # .place_snap("z-", AnchorRef(scene.children["A"], "z+"))
    ))
    
    return scene

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
    scene.add_child("line", (
        LineDomino(
            scene.anchor(np.array([0, 1, 0])),
            scene.anchor(np.array([0, 0.15, 0]))
        ).trigger()
    ))
    return scene

def main() -> None:
    scene = build_scene_4()
    xml = CompileWorld(scene)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        configure_camera(viewer.cam)
        while viewer.is_running():
            start_time = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            end_time = time.time()
            dt = end_time - start_time
            if dt < model.opt.timestep:
                time.sleep(model.opt.timestep * SLOW_DOWN_FACTOR - dt)

if __name__ == "__main__":
    main()