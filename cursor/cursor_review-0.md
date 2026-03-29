# Review Findings

## Findings

### 1. `component.py` currently has invalid function signatures and will not import

`move_to_touch()` and `rotate_to_touch()` place a non-default parameter (`target`) after defaulted parameters, which is a Python syntax error. As written, importing `Domino/components/component.py` should fail before any of the placement code can run.

```44:50:Domino/components/component.py
    def move_to_touch(self, axis: np.ndarray = np.array([1, 0, 0]), target: Component) -> "Component":
        # TODO Project all basic dominoes onto axis, find the min slide distance and apply.
        # NOTE In order to iterate all basic dominoes, we need to DFS self.chilren recursively, and manage poses along the way.
        pass
        return self
    
    def rotate_to_touch(self, anchor: np.ndarray = np.array([0, 0, 0]), axis: np.ndarray = np.array([1, 0, 0]), target: Component) -> "Component":
```

Suggested fix:
- reorder parameters so `target` comes before defaulted ones, or make `target` keyword-only

### 2. `place()` cannot work with the current anchor type, and its translation solve is mathematically wrong

`self.anchors[...]` stores raw `np.ndarray` points, but `place()` calls `.inverse()` on the selected anchor. That will raise `AttributeError` immediately. Even beyond that, the solve should depend on the chosen component rotation: for a local anchor point `p`, the correct translation is `target_position - rotation @ p`.

```11:13:Domino/components/component.py
        self.anchors = {
            "center": np.array([0, 0, 0])
        }
```

```17:31:Domino/components/component.py
    def place(
            self,
            anchor_name: str = "center",
            target_position: np.ndarray = np.array([0, 0, 0]),
            rotation: np.ndarray = np.eye(3),
    ) -> "Component":
        # First apply rotation.
        self.from_parent.set_rotation(rotation)
        # Then apply position with anchor definitions.
        anchor_from_self = self.anchors[anchor_name]
        # We want (under parent's frame):
        #   (anchor_from_self * self.from_parent).position == target_position
        # which gives (left multiply by anchor_from_self.inverse()):
        #   self.from_parent.position = anchor_from_self.inverse() * target_position
        self.from_parent.set_position(anchor_from_self.inverse() * target_position)
```

Suggested fix:
- either make anchors proper `Pose`/frame objects, or keep them as points and compute `target_position - rotation @ local_anchor`

### 3. `Pose.inverse()` is incorrect for any rotated pose

The inverse translation of a rigid transform should be `-R^T p`, not `R (-p)`. The current implementation only works when `rotation` is identity. Any caller that relies on `inverse()` will get wrong results once rotation is present.

```37:38:Domino/geometry/pose.py
    def inverse(self) -> 'Pose':
        return Pose(self.rotation @ -self.position, self.rotation.T)
```

Suggested fix:
- change the translation term to `self.rotation.T @ (-self.position)`

### 4. `rotate()` overwrites the component pose instead of rotating the existing pose around the pivot

The current method computes `translation * rotation * translation.inverse()` and assigns that directly to `self.from_parent`. This throws away the previous pose instead of composing with it. It also uses `anchor - self.from_parent.position` as the pivot translation, which is not the correct parent-space transform for "rotate around this anchor".

```38:42:Domino/components/component.py
    def rotate(self, anchor: np.ndarray, axis: np.ndarray, angle: float) -> "Component":
        translation = Pose(position = anchor - self.from_parent.position)
        rotation = Pose(rotation = rotation_matrix_from_axis_angle(axis, angle))
        self.from_parent = translation * rotation * translation.inverse()
        return self
```

Suggested fix:
- compose the new transform with the existing pose instead of replacing it
- implement pivot rotation as `T(anchor) * R * T(-anchor) * current_pose` in the correct frame

### 5. Shared mutable defaults will leak state across instances

`Pose()` and several `np.ndarray` defaults are created once at function definition time and then shared. Mutating `self.from_parent` on one component can accidentally affect later instances created with the default constructor.

```16:23:Domino/geometry/pose.py
class Pose:
    def __init__(
            self,
            position: np.ndarray = np.array([0, 0, 0]),
            rotation: np.ndarray = np.eye(3)
    ):
        # TODO Check if position and rotation are valid.
        self.position = position
        self.rotation = rotation
```

```7:8:Domino/components/component.py
class Component:
    def __init__(self, from_parent: Pose = Pose()):
```

```17:21:Domino/components/component.py
    def place(
            self,
            anchor_name: str = "center",
            target_position: np.ndarray = np.array([0, 0, 0]),
            rotation: np.ndarray = np.eye(3),
```

Suggested fix:
- default these parameters to `None` and allocate fresh arrays / `Pose` objects inside the function body

### 6. `rotation_matrix_from_axis_angle()` is unsafe and appears to use the opposite sign convention

The helper never normalizes `axis`, so a non-unit axis can produce a non-orthonormal matrix. Also, the skew-symmetric term is built from `np.cross(axis, np.eye(3))`, which for a simple axis like `z+` yields the opposite sign from the standard right-handed Rodrigues formula.

```3:6:Domino/geometry/pose.py
def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    R = np.eye(3)
    R[:3, :3] = np.cos(angle) * np.eye(3) + np.sin(angle) * np.cross(axis, np.eye(3)) + (1 - np.cos(angle)) * np.outer(axis, axis)
    return R
```

Suggested fix:
- normalize `axis`
- build the skew matrix explicitly with the standard Rodrigues sign convention

## Improvement Opportunities

### 7. `Domino.from_rpy()` duplicates `rotation_matrix_from_rpy()`

There are currently two independent implementations of roll-pitch-yaw rotation construction. That creates an unnecessary maintenance risk if one changes and the other does not.

```81:86:Domino/components/component.py
    @classmethod
    def from_rpy(cls, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
        R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return R_x @ R_y @ R_z
```

Suggested fix:
- keep a single implementation in `geometry/pose.py` and reuse it from `Domino`

## Residual Risk

Even after fixing the syntax and math bugs above, the current placement API still mixes point anchors, full poses, and frame transforms somewhat implicitly. It would be worth deciding early whether an anchor is:

- just a local point, or
- a full local frame (`Pose`)

because `place()`, `rotate()`, and future touch-based operations all get much clearer once that choice is explicit.
