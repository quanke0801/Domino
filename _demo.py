import time
from pathlib import Path

import mujoco
import mujoco.viewer
from PIL import Image


OUTPUT_DIR = Path(r"frames")
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 60
DURATION_SECONDS = 5.0


def build_domino_scene_xml(
    count: int = 18,
    spacing: float = 0.045,
    lean_deg: float = 12.0,
) -> str:
    domino_size = (0.004, 0.02, 0.03)  # half-sizes: x, y, z
    z = domino_size[2]

    bodies = []
    for i in range(count):
        x = i * spacing
        euler = f' euler="0 {lean_deg} 0"' if i == 0 else ""
        bodies.append(
            f"""
      <body name="domino_{i}" pos="{x:.4f} 0 {z:.4f}"{euler}>
        <freejoint/>
        <geom
            name="domino_geom_{i}"
            type="box"
            size="{domino_size[0]} {domino_size[1]} {domino_size[2]}"
            density="700"
            friction="0.8 0.02 0.001"
            rgba="0.85 0.25 0.2 1"/>
      </body>"""
        )

    domino_bodies = "\n".join(bodies)
    return f"""
<mujoco model="domino_line">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <visual>
    <global offwidth="{FRAME_WIDTH}" offheight="{FRAME_HEIGHT}"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.15 0.15 0.15"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2"/>
    <material name="grid" texture="grid" texrepeat="6 6" texuniform="true" reflectance="0.05"/>
  </asset>

  <worldbody>
    <light pos="0 0 2"/>
    <geom name="ground" type="plane" pos="0 0 0" size="2 2 0.1" material="grid"
          friction="1.0 0.05 0.01"/>
{domino_bodies}
  </worldbody>
</mujoco>
"""


def configure_camera(camera: mujoco.MjvCamera) -> None:
    camera.azimuth = 90
    camera.elevation = -20
    camera.distance = 0.7
    camera.lookat[:] = [0.35, 0.0, 0.05]


def main(gui: bool = True, dump_folder: Path | str | None = None) -> None:
    xml = build_domino_scene_xml()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    frame_count = int(FPS * DURATION_SECONDS)
    dump_path = Path(dump_folder) if dump_folder is not None else None
    renderer = None

    if dump_path is not None:
        dump_path.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=FRAME_HEIGHT, width=FRAME_WIDTH)

    if gui:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            configure_camera(viewer.cam)
            next_dump_time = 1.0 / FPS
            frame_idx = 0

            while viewer.is_running():
                # In passive mode, the viewer toggles Simulate.run via keyboard
                # controls (space pause/resume, backspace reset).
                sim = viewer._get_sim()  # pylint: disable=protected-access
                running = sim is None or bool(sim.run)

                if running:
                    step_start = time.perf_counter()
                    mujoco.mj_step(model, data)

                    if renderer is not None and dump_path is not None:
                        while data.time >= next_dump_time:
                            renderer.update_scene(data, camera=viewer.cam)
                            rgb = renderer.render()
                            Image.fromarray(rgb).save(dump_path / f"{frame_idx}.png")
                            frame_idx += 1
                            next_dump_time = (frame_idx + 1) / FPS

                    sleep_time = model.opt.timestep - (time.perf_counter() - step_start)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                else:
                    mujoco.mj_forward(model, data)
                    time.sleep(0.01)

                viewer.sync()
    else:
        camera = mujoco.MjvCamera()
        configure_camera(camera)

        for frame_idx in range(frame_count):
            target_time = (frame_idx + 1) / FPS
            while data.time < target_time:
                mujoco.mj_step(model, data)

            if renderer is not None and dump_path is not None:
                renderer.update_scene(data, camera=camera)
                rgb = renderer.render()
                Image.fromarray(rgb).save(dump_path / f"{frame_idx}.png")

    if renderer is not None:
        renderer.close()


if __name__ == "__main__":
    main()
