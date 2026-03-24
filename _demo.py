from pathlib import Path

import mujoco
from PIL import Image


OUTPUT_DIR = Path(r"d:\Code\BulletDomino\MujocoDomino\frames")
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


def main() -> None:
    xml = build_domino_scene_xml()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    camera = mujoco.MjvCamera()
    camera.azimuth = 90
    camera.elevation = -20
    camera.distance = 0.7
    camera.lookat[:] = [0.35, 0.0, 0.05]

    renderer = mujoco.Renderer(model, height=FRAME_HEIGHT, width=FRAME_WIDTH)
    frame_count = int(FPS * DURATION_SECONDS)

    for frame_idx in range(frame_count):
        target_time = (frame_idx + 1) / FPS
        while data.time < target_time:
            mujoco.mj_step(model, data)

        renderer.update_scene(data, camera=camera)
        rgb = renderer.render()
        Image.fromarray(rgb).save(OUTPUT_DIR / f"{frame_idx}.png")

    renderer.close()


if __name__ == "__main__":
    main()
