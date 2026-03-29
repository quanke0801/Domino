# Placement API Simplification

This document responds to `feedback-1.md`.

## Main Decision

`place()` should be simple and exact. It should not accept an arbitrary bag of constraints.

Instead, `place()` should always take:

- exactly one 3-DoF position spec
- exactly one 3-DoF rotation spec

and from those two pieces compute exactly one `from_parent` pose.

That keeps the API easy to reason about and avoids the "are these constraints insufficient / conflicting / overdetermined?" problem.

## Recommended Shape

Conceptually:

```python
component.place(position=..., rotation=...)
```

where:

- `position` fully determines translation
- `rotation` fully determines orientation

Both should resolve in the parent frame, and the result is stored only in `component.from_parent`.

## Position Spec

For v1, support only these forms.

### 1. Anchor-to-anchor coincidence

Example:

```python
brick.place(
    position=coincide("D", other.anchor("T")),
    rotation=preset("standing"),
)
```

Meaning:

- choose one local anchor on the current component
- choose one target anchor on another component
- compute translation so those anchor origins coincide

Rotation is handled entirely separately.

### 2. Explicit coordinates

Example:

```python
brick.place(
    position=at(x=0.1, y=0.2, z=0.05),
    rotation=rpy(0, 0, 90),
)
```

Meaning:

- position is directly given in parent coordinates

## Rotation Spec

For v1, support only these forms.

### 1. Preset orientation

Examples:

```python
rotation=preset("standing")
rotation=preset("sideways_x")
rotation=preset("sideways_y")
```

This is likely the most common case for domino authoring.

### 2. Copy another component's rotation

Example:

```python
rotation=copy_rotation(other)
```

Meaning:

- copy only orientation
- do not copy position

This is useful when growing structures relative to siblings.

### 3. Explicit `rpy`

Example:

```python
rotation=rpy(roll=0, pitch=0, yaw=45)
```

Meaning:

- orientation is directly specified

## Why This Split Works Well

This gives exactly 6 DoF:

- 3 from position
- 3 from rotation

So `place()` is not a solver. It is just pose construction.

That is a very good design for v1 because:

- behavior is obvious
- implementation is easy
- debugging is easy
- storage stays clean
- you can still author components relative to siblings

## Anchor Semantics Under This Design

An anchor should still be a local frame, not just a point.

But under this simpler v1 API:

- `position` uses only the anchor origin
- `rotation` is resolved separately

So even if an anchor has axes and normals, `coincide(...)` only consumes its point for now.

Later, if desired, you can add:

- `copy_rotation(other.anchor("..."))`
- `align_rotation_to_anchor(...)`

But those should be separate rotation spec types, not bundled into `coincide(...)`.

## Recommended API Sketch

```python
class Component:
    def place(self, position, rotation) -> "Component":
        ...
        return self
```

Position specs:

```python
at(x, y, z)
coincide(local_anchor_name, target_anchor)
```

Rotation specs:

```python
preset(name)
copy_rotation(other_component)
rpy(roll, pitch, yaw)
```

This keeps the authoring surface very small.

## Example Usage

### Put a standing brick on top of another brick

```python
Brick().place(
    position=coincide("D", base.anchor("T")),
    rotation=preset("standing"),
)
```

### Put a sideways brick on the ground

```python
Brick().place(
    position=coincide("D", ground.anchor("T")),
    rotation=preset("sideways_x"),
)
```

### Place a brick at xyz with same rotation as sibling

```python
Brick().place(
    position=at(0.2, 0.1, 0.0),
    rotation=copy_rotation(other),
)
```

### Put a brick's left face center at another component's right face center

```python
Brick().place(
    position=coincide("L", other.anchor("R")),
    rotation=preset("standing"),
)
```

## How To Support "Move Until Touch"

Do not make "move until touch" part of `place()`.

That is the cleanest answer.

`place()` should define a complete pose directly.

Touch-based operations should be separate post-placement adjustment operations that solve only one scalar DoF at a time.

Recommended design:

```python
component.place(position=..., rotation=...)
component.slide_until_touch(...)
component.drop_until_touch(...)
```

or more generically:

```python
component.move_until_touch(axis=..., feature=..., target=...)
```

## Why Touch Should Be Separate

If you allow both:

- unconstrained placement
- rotate until touch
- move until touch

inside one general solver, then the system can easily become ambiguous:

- multiple valid solutions
- no valid solution
- order-dependent behavior

That is exactly the chaos you want to avoid.

## Recommended Rule For Touch Operations

Each touch operation may solve at most one scalar parameter.

Good examples:

- translate along `+x` until touch
- translate along `-z` until touch
- rotate around local `z` until touch

Bad examples for v1:

- simultaneously move and rotate until touch
- solve 2 or 3 axes at once until first contact
- combine multiple touch conditions in one operation

So the rule should be:

- one operation
- one free scalar variable
- one contact target

That keeps the result well-defined.

## Suggested Touch API

### Translation-only touch

```python
component.move_until_touch(
    axis="-x",
    moving_anchor="L",
    target=other,
    gap=0.0,
)
```

or

```python
component.move_until_touch(
    axis="-z",
    target=ground,
)
```

This should:

- keep rotation fixed
- keep the two orthogonal translation coordinates fixed
- solve only the translation distance along one axis

### Rotation-only touch

Possible later:

```python
component.rotate_until_touch(
    axis="z",
    pivot="C",
    target=other,
)
```

But this should be added only if you truly need it.

My recommendation:

- implement translation-only touch first
- delay rotation-until-touch until you have a real use case

## Composition Rule

If both are ever needed, require them to be written explicitly in sequence:

```python
brick.place(...)
brick.move_until_touch(axis="-x", target=other)
brick.rotate_until_touch(axis="z", target=other2)
```

This makes the order visible and intentional.

Even then, the behavior is path-dependent, which is fine as long as it is explicit.

What you should avoid is:

```python
brick.place(..., move_until_touch=..., rotate_until_touch=...)
```

because that implies a joint solver problem instead of a simple placement API.

## Relationship To `from_parent`

This design still fits the earlier decision perfectly:

- the only stored transform is `from_parent`
- `place()` computes it directly
- `move_until_touch()` updates it by a one-DoF adjustment
- world pose is always derived from the tree

So there is still only one source of truth.

## Recommended V1 Surface Area

Keep v1 extremely small:

- `Component.place(position, rotation)`
- `at(x, y, z)`
- `coincide(local_anchor, target_anchor)`
- `preset(name)`
- `copy_rotation(component)`
- `rpy(roll, pitch, yaw)`
- `move_until_touch(axis, target, gap=0)`
- `drop_until_touch(target=ground, gap=0)` as a convenience wrapper over `move_until_touch(axis="-z", ...)`

That is enough to build a lot of structures without turning placement into a solver framework.

## Final Recommendation

Use this split:

- `place()` is exact 6-DoF pose definition
- touch methods are separate 1-DoF adjustment operators

This gives you:

- simple implementation
- clear mental model
- no under/over-constrained placement logic
- room to grow later if you need more advanced placement

Most importantly, it matches your stated preference to start simple without painting yourself into a corner.
