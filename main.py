import time
import logging
logging.basicConfig(level=logging.INFO)

import numpy as np
import mujoco
import mujoco.viewer

from Domino.components import *
from Domino.geometry.pose import Pose
from Domino.scenes.test_scenes import *
from Domino.scenes.demo_scenes import *

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
MATERIALS = [
f"""<material name="domino_mat_{i}"
    rgba="{color} 1"
    specular="0.35"
    shininess="0.2"
    reflectance="0.1"
/>""" for i, color in enumerate(PALETTE)
]

SEED = 48
np.random.seed(SEED)
SLOW_DOWN_FACTOR = 2
TIME_STEP = 0.005

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
                name="collision_{body_id}"
                type="box"
                size="{Domino.SIZE[0] / 2} {Domino.SIZE[1] / 2} {Domino.SIZE[2] / 2}"
                density="1000"
                friction="0.25 0.005 0.0001"
                group="1"
            />
            <geom
                name="visual_{body_id}"
                type="mesh"
                mesh="domino_bevel_mesh"
                material="domino_mat_{np.random.randint(len(PALETTE))}"
                contype="0"
                conaffinity="0"
                group="0"
            />
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
    <geom solref="{TIME_STEP * 5} 1" solimp="0.99 0.99 0.002" friction="1 0.005 0.0001"/>
  </default>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.15 0.15 0.15"/>
  </visual>
  <asset>
    <mesh name="domino_bevel_mesh" file="Domino/assets/domino_bevel.obj"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2"/>
    <material name="grid" texture="grid" texrepeat="6 6" texuniform="true" reflectance="0.05"/>
    {"\n".join(MATERIALS)}
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
    camera.elevation = -45
    camera.distance = 2
    # camera.distance = 0.5
    camera.lookat[:] = [0, 0, 0]


def main() -> None:
    # scene = scene_condition_gate_run()
    scene = test_scene_copy()
    xml = CompileWorld(scene)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.opt.geomgroup[1] = 0
        configure_camera(viewer.cam)
        while viewer.is_running():
            # viewer.cam.azimuth += 360 / 5 * TIME_STEP
            start_time = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            end_time = time.time()
            dt = end_time - start_time
            if dt < model.opt.timestep:
                time.sleep(model.opt.timestep * SLOW_DOWN_FACTOR - dt)
            else:
                print(f"Delay detected: {dt - model.opt.timestep}")
    # mujoco.viewer.launch(model, data, show_left_ui=False, show_right_ui=False)

if __name__ == "__main__":
    main()