# Review Findings

## Findings

### 1. `Component.rotate()` still does not rotate around the requested parent-frame anchor correctly

The previous version improved because it now composes with `self.from_parent`, but the pivot transform is still built from `anchor - self.from_parent.position`, which is not the same as `T(anchor) * R * T(-anchor)`. This will only be correct in limited cases and will give the wrong result once the component already has a nontrivial pose.

```34:38:Domino/components/component.py
    def rotate(self, anchor: np.ndarray, axis: np.ndarray, angle: float) -> "Component":
        translation = Pose(position = anchor - self.from_parent.position)
        rotation = Pose(rotation = rotation_matrix_from_axis_angle(axis, angle))
        self.from_parent = translation * rotation * translation.inverse() * self.from_parent
        return self
```

Minimal-change fix:

```python
def rotate(self, anchor: np.ndarray, axis: np.ndarray, angle: float) -> "Component":
    translation = Pose(position = anchor)
    rotation = Pose(rotation = rotation_matrix_from_axis_angle(axis, angle))
    self.from_parent = translation * rotation * translation.inverse() * self.from_parent
    return self
```

That gives the standard parent-frame pivot rotation:

- translate to pivot
- rotate
- translate back
- apply to current pose

### 2. `Pose` still uses shared mutable default arrays

`np.array([0, 0, 0])` and `np.eye(3)` are created once at function definition time, then shared across instances created with the default constructor. This can leak state between `Pose()` objects if any code mutates the arrays in place.

```16:24:Domino/geometry/pose.py
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

Minimal-change fix:

```python
class Pose:
    def __init__(
            self,
            position: np.ndarray | None = None,
            rotation: np.ndarray | None = None
    ):
        # TODO Check if position and rotation are valid.
        self.position = np.array([0, 0, 0]) if position is None else position
        self.rotation = np.eye(3) if rotation is None else rotation
```

### 3. `Component.place()` still uses shared mutable default arrays

This is the same default-argument problem as above. It is lower risk here because the current method replaces the arrays rather than mutating them in place, but it is still a footgun and easy to avoid now.

```17:24:Domino/components/component.py
    def place(
            self,
            anchor_name: str = "center",
            # Target position in parent frame.
            target_position: np.ndarray = np.array([0, 0, 0]),
            # Desired self_from_parent rotation.
            rotation: np.ndarray = np.eye(3),
    ) -> "Component":
```

Minimal-change fix:

```python
def place(
        self,
        anchor_name: str = "center",
        # Target position in parent frame.
        target_position: np.ndarray | None = None,
        # Desired self_from_parent rotation.
        rotation: np.ndarray | None = None,
) -> "Component":
    if target_position is None:
        target_position = np.array([0, 0, 0])
    if rotation is None:
        rotation = np.eye(3)
    # Apply rotation first, then position.
    self.from_parent.set_rotation(rotation)
    self.from_parent.set_position(target_position - rotation @ self.anchors[anchor_name])
    return self
```

### 4. `rotation_matrix_from_axis_angle()` will divide by zero for a zero-length axis

You fixed the sign convention and normalization, but a zero vector now produces a divide-by-zero and contaminates the matrix with `nan`s. Since `rotate()` accepts an arbitrary `axis`, this should be guarded explicitly.

```3:7:Domino/geometry/pose.py
def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    skew_axis = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.cos(angle) * np.eye(3) + np.sin(angle) * skew_axis + (1 - np.cos(angle)) * np.outer(axis, axis)
    return R
```

Minimal-change fix:

```python
def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("axis must be non-zero")
    axis = axis / axis_norm
    skew_axis = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.cos(angle) * np.eye(3) + np.sin(angle) * skew_axis + (1 - np.cos(angle)) * np.outer(axis, axis)
    return R
```

## Improvement Opportunities

### 5. `Pose` stores caller-owned arrays directly

Even once the defaults are fixed, `self.position = position` and `self.rotation = rotation` still alias the caller's arrays. That may be fine if it is intentional, but if you want `Pose` to own its data, copying on assignment will make bugs much less surprising.

```22:24:Domino/geometry/pose.py
        # TODO Check if position and rotation are valid.
        self.position = position
        self.rotation = rotation
```

Minimal-change fix:

```python
        self.position = np.array(position, copy = True)
        self.rotation = np.array(rotation, copy = True)
```

### 6. `Pose.__mul__()` silently returns `None` for unsupported types

Right now, if `other` is neither `Pose` nor `np.ndarray`, the method falls off the end and returns `None`. Raising a clear error will make bugs easier to catch.

```32:36:Domino/geometry/pose.py
    def __mul__(self, other: Pose | np.ndarray):
        if isinstance(other, Pose):
            return Pose(self.position + self.rotation @ other.position, self.rotation @ other.rotation)
        elif isinstance(other, np.ndarray):
            return self.position + self.rotation @ other
```

Minimal-change fix:

```python
def __mul__(self, other: Pose | np.ndarray):
    if isinstance(other, Pose):
        return Pose(self.position + self.rotation @ other.position, self.rotation @ other.rotation)
    elif isinstance(other, np.ndarray):
        return self.position + self.rotation @ other
    raise TypeError(f"unsupported operand type: {type(other)}")
```

## Summary

The important math fixes from the first review are mostly in place:

- `place()` no longer calls `.inverse()` on a raw anchor point
- `Pose.inverse()` is now correct
- `rotation_matrix_from_axis_angle()` now uses the standard Rodrigues form
- the duplicate `from_rpy()` implementation is gone
- the invalid `move_to_touch()` / `rotate_to_touch()` signatures are fixed

The main remaining correctness issue is `Component.rotate()`. After that, the most worthwhile cleanup is removing the mutable default arrays and adding a zero-axis guard in `rotation_matrix_from_axis_angle()`.
