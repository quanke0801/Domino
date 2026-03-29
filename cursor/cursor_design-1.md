# Component And Placement Design Follow-Up

This document responds specifically to the feedback in `feedback-0.md`. The main topic is not MuJoCo export, but how the Python-side `Component` tree and authoring API should work so that building structures is concise, relative, and extensible.

## Summary Of The Recommendation

- Keep the component tree.
- Store only one source of truth for each node's pose: a relative transform from its parent.
- Do not eagerly store world coordinates on components.
- Support sibling-relative authoring by expressing placement as a deferred relation that computes the child's `from_parent` transform.
- Make creation concise by combining:
  - a small set of primitive component constructors
  - a unified anchor-based placement API
  - optional orientation presets
- Treat faces, edges, corners, and sockets as named anchors, not raw coordinates.

The key design choice is:

- components store structure
- placement constraints derive relative transforms

That gives single-source-of-truth storage without forcing the user to think only in parent-local coordinates.

## 1. Should The Tree Store Only `from_parent`?

Yes. Each component should store only:

- `parent`
- `children`
- `from_parent: Pose`
- local geometry description
- local named anchors and sockets

It should not permanently store:

- absolute/world pose
- sibling-relative pose
- duplicated cached transforms as authoritative data

World pose should be derived by walking up the tree:

```python
world_pose(component) = world_pose(component.parent) * component.from_parent
```

This is the cleanest design because:

- there is only one authoritative transform per component
- moving a parent automatically moves the whole subtree
- serialization and debugging stay simple
- export to MJCF or any other backend is straightforward

## 2. But How Do We Place A Child Relative To A Sibling?

This is the important part: the API the user writes does not need to match the storage format.

The user should be allowed to say:

```python
root.add("a", Brick.standing())
root.add("b", Brick.standing())
root["b"].place(
    Match("left", of=root["a"].anchor("right"), gap=0.0)
)
```

Even though `b` stores only `from_parent`, the system can compute it by:

1. resolving the target anchor of sibling `a` into the parent coordinate frame
2. resolving the local anchor of `b`
3. solving the transform needed so `b.local_anchor` matches the target anchor
4. writing the result into `b.from_parent`

So the design should be:

- storage model: only `from_parent`
- authoring model: placement relations between anchors

This removes duplication while still allowing "grow the structure from existing parts".

## 3. Unified Placement Model

The API should not be a bag of ad-hoc methods like `put_on_top_of`, `slide_left_until_touch`, `make_perpendicular_above`, and so on.

Instead, all of those should be special cases of one general concept:

- place one component by matching one or more named anchors, optionally with offsets and orientation constraints

## 4. Core Concepts

### `Pose`

A rigid transform:

- position
- rotation

### `Anchor`

A named local frame attached to a component.

An anchor is not just a point. It should contain:

- origin
- orientation
- optional outward normal
- optional tangent/binormal directions

This is important because "aligning" usually means both:

- move point A to point B
- orient frame A relative to frame B

### `AnchorQuery`

A reference to an anchor on an existing component:

```python
root["pile"].anchor("T")
root["brick3"].anchor("RF")
```

### `Placement`

A declarative relation that says how a component should be placed relative to another anchor/frame/object.

Examples:

- match anchor to anchor
- offset by some distance along a target frame axis
- align only some axes
- slide until touching
- drop until touching

## 5. Recommended Anchor Naming

You asked for common anchor points to be accessible by names rather than coordinates. That is a very good idea.

Use a compact but systematic naming scheme.

For primitive box-like domino bricks, recommend these anchor families:

- `C`: center
- `T`, `D`, `L`, `R`, `F`, `B`: face centers
- `TF`, `TB`, `TL`, `TR`, etc.: edge midpoints
- `TFR`, `TFL`, `DBR`, etc.: corners

Where:

- `T` = top
- `D` = down
- `L` = left
- `R` = right
- `F` = front
- `B` = back

Examples:

- `anchor("T")`: top face center
- `anchor("DL")`: midpoint of the bottom-left edge
- `anchor("TFR")`: top-front-right corner

This scheme is compact, easy to generate, and easy to memorize.

Also provide semantic aliases when useful:

- `top` -> `T`
- `bottom` -> `D`
- `left` -> `L`
- `right` -> `R`
- `front` -> `F`
- `back` -> `B`
- `center` -> `C`

## 6. The One General Placement Operation

Every component should support a single main operation conceptually like:

```python
component.place(*constraints)
```

Where each constraint expresses one part of the placement intention.

For example:

```python
brick.place(
    coincide("D", other.anchor("T")),
    orient(z_to=other.anchor("T").normal),
    yaw(90),
)
```

or more compactly:

```python
brick.place(
    match("D", to=other.anchor("T"), rx=0, ry=0, rz=90)
)
```

The exact syntax can vary, but the idea is:

- one general placement entry point
- a small vocabulary of reusable constraints

## 7. Constraint Vocabulary

The following few constraint types are enough to cover most needs.

### `coincide(local_anchor, target_anchor)`

Move the component so that the local anchor origin matches the target anchor origin.

### `orient(...)`

Specify orientation alignment, for example:

- local up matches target normal
- local front matches target tangent
- apply extra yaw/pitch/roll after alignment

### `offset(dx, dy, dz, frame="target" | "local" | "parent" | "world")`

Apply a translation after anchor matching.

### `slide(local_anchor, against=target_anchor, axis=..., gap=0)`

Translate along one axis until the specified feature touches another feature.

### `drop(local_anchor, onto=target_component_or_anchor, axis="z-", gap=0)`

Move downward until contact with support geometry.

### `rotate(...)`

Apply additional local rotation around selected axes.

These are generic enough that many specific authoring needs become combinations of them.

## 8. How Your Example Cases Fit The Unified Model

### Case A

"Put a standing domino on top of a lying domino, align their xy centers, maybe rotate 90 degrees around z."

This becomes:

```python
top_brick.place(
    coincide("D", base_brick.anchor("T")),
    orient(local="+z", to=base_brick.anchor("T").normal),
    rotate(z=90),
)
```

This is not a special `put_on_top_of_laying_domino()` method. It is just anchor matching plus orientation.

### Case B

"Put a domino sideways on ground."

This becomes:

```python
brick.place(
    coincide("D", scene.ground.anchor("T")),
    preset("sideways"),
)
```

Or:

```python
brick = Brick.sideways()
brick.place(coincide("D", scene.ground.anchor("T")))
```

The "sideways" part should usually be a predefined orientation preset, not a placement method.

### Case C

"Put a standing domino on ground at xyz and slide it toward x- until it touches another component."

This becomes:

```python
brick.place(
    preset("standing"),
    at(x, y, z0),
    drop("D", onto=scene.ground),
    slide("L", against=other.anchor("R"), axis="-x", gap=0),
)
```

Again, this is still the same framework:

- choose orientation
- choose initial reference placement
- apply drop
- apply slide/contact alignment

## 9. Concise One-Line Creation

You want creation to be concise because structures like `Crossing` may have many bricks.

That is a strong argument for making primitive constructors expressive, not for reducing flexibility.

Recommended pattern:

```python
self.add("b0", Brick.standing().place(...))
self.add("b1", Brick.sideways().place(...))
self.add("b2", Brick.standing().place(...))
```

So each brick can be:

- created in one line
- configured with a preset orientation
- placed relative to an existing component with one `.place(...)` call

This is much better than:

- manually constructing raw transforms
- manually computing half-heights and trigonometric offsets
- adding dozens of placement helper methods

## 10. Suggested `Brick` Presets

Primitive domino creation should start with only a few orientation presets:

- `Brick.standing()`
- `Brick.sideways_x()`
- `Brick.sideways_y()`
- `Brick.flat_x()`
- `Brick.flat_y()`

Or, if you prefer fewer but clearer names:

- `Brick.standing()`
- `Brick.sideways()`
- `Brick.lying()`

with optional explicit axis selection:

```python
Brick.sideways(axis="x")
Brick.sideways(axis="y")
```

These presets solve the common case compactly while still allowing arbitrary rotation later.

## 11. Recommended `Component` API Shape

Here is a recommended conceptual API.

### Structure

```python
class Component:
    name: str
    parent: Component | None
    children: dict[str, Component]
    from_parent: Pose
    sockets: dict[str, Anchor]
    tags: set[str]
```

### Required methods

- `add(name, child)`
- `anchor(name)`
- `socket(name)`
- `world_pose()`
- `place(*constraints)`
- `bounds()`
- `copy()`

### Optional convenience methods

- `move(...)`
- `rotate(...)`
- `tag(...)`

But the main geometry-authoring method should remain `place(...)`.

## 12. Parent-Relative Storage With Sibling-Relative Authoring

This is the core answer to the first feedback point.

The flow should be:

1. User adds child `b` under parent `p`.
2. User writes a placement relation that references sibling `a`.
3. The system resolves `a`'s target anchor into `p` coordinates.
4. The system computes `b.from_parent`.
5. Only `b.from_parent` is stored.

This gives:

- no duplicated transforms
- easy subtree movement
- flexible relative authoring
- clean export

## 13. Should Placement Be Immediate Or Deferred?

Recommendation:

- start with immediate resolution for simple cases
- keep the API compatible with deferred resolution later

Immediate resolution means:

- when `.place(...)` is called, compute `from_parent` now

This is easiest if:

- the referenced siblings already exist
- bounds and anchors are available immediately

Deferred resolution may become useful later for:

- cyclic dependencies
- multi-step layout solving
- bulk rebuild after parameter edits

But for v1, immediate resolution is probably enough and much simpler.

## 14. Bounds And Touching

To support "slide until touch" and "drop until touch", every primitive and composite component needs a bounding representation.

Recommended rule:

- primitives provide exact local bounds
- composite components compute bounds as the union of children

For v1, use axis-aligned bounds in the parent or world frame. That is enough for many placement operations.

Later, if needed, add:

- oriented bounds
- support surfaces
- more exact shape queries

This is a good incremental path.

## 15. Sockets Versus Anchors

Keep both concepts, but make their relationship explicit.

- `Anchor`: any named geometric reference frame on a component
- `Socket`: a special anchor intended for logical connection to another component

So:

- all sockets are anchors
- not all anchors are sockets

Examples:

- `T`, `L`, `R`, `TFR` are geometric anchors
- `in`, `out`, `trigger`, `branch_left` are semantic sockets

This lets the same placement engine work for both geometry and logical composition.

## 16. How A `Crossing` Component Should Be Written

A `Crossing` should not manually compute world coordinates for ten bricks.

Instead:

- create each brick in one line with an orientation preset
- place each brick relative to earlier bricks or local guide anchors
- expose only the meaningful sockets of the composite

Example style:

```python
self.add("base0", Brick.standing().place(...))
self.add("base1", Brick.standing().place(...))
self.add("lean0", Brick.sideways().place(...))
self.add("bridge0", Brick.standing().place(...))
```

If some arrangement is inherently complicated, that complexity should live in:

- the relationships chosen in `.place(...)`

not in:

- repeated raw trigonometry
- hand-managed center coordinates

## 17. Extensibility

This design is expandable because new authoring behavior can be added as new constraints, not new special-case component methods.

Examples of future additions:

- `tilt_until(...)`
- `lean_against(...)`
- `snap_min_gap(...)`
- `place_by_solver(...)`

They can all plug into:

```python
component.place(...)
```

without changing the storage model.

## 18. Final Recommendation

Use this mental model:

- the component tree stores only parent-relative transforms
- the user authors structures by referencing named anchors on existing components
- placement is expressed through a small, general constraint language
- concise one-line brick creation comes from expressive constructors plus `.place(...)`

If you follow that, you get all of these at once:

- single-source-of-truth transform storage
- sibling-relative and structure-growth authoring
- concise component code
- named faces/edges/corners
- a design that stays clean as the project grows

## 19. Concrete V1 Recommendation

If you want a minimal but strong v1, implement only these first:

- `Pose`
- `Anchor`
- `Component.from_parent`
- primitive box anchors with `C`, `T`, `D`, `L`, `R`, `F`, `B`, edges, corners
- `Brick.standing()` and `Brick.sideways(...)`
- `place(coincide(...), orient(...), offset(...))`
- `drop(...)`
- `slide(...)`

That set is already enough to build many domino structures in a clean way without exploding the API surface.
