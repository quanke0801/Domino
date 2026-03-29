This project is being re-design and re-implementation with Mujoco from scratch. Previous version of pybullet domino simulation is moved to `_deprecated_pybullet/` folder.

# TODO
- Features
    - Component should also support move_to_touch and rotate_to_touch.
    - Change slide_t to move_distance (discard the concept of slide velocity).
    - Find a good name for shaft-support-trigger structure, and make component.
    - Unify ground as a component. Use singleton.
- Code
    - Add PointRef (as previously AnchorRef) and VectorRef so that we don't need conventions to constraint arg frame as in `axis_in_parent`.
    - Add unit tests.
