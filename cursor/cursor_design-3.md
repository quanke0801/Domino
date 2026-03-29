# Anchor And Snap Capability Design

This document addresses whether anchors and snap-like operations should live on `Component`, on `Domino`, or somewhere in between.

## Short Answer

Do not keep full anchor/snap behavior on every `Component`.

But also do not demote it all the way down to `Domino` only.

The best split is:

- `Component` keeps only tree structure, transform, and sockets.
- A separate geometric capability layer provides anchors and snap/edit operations.
- `Domino` implements that capability fully.
- Compound components opt in only when they want to expose public geometric anchors.

That gives you:

- a simple generic base class
- concise brick authoring
- support for exceptional compound cases like `PileDomino`
- no pressure to pretend every compound component is a geometric editing target

## Why `Component` Should Stay Small

Your usage pattern is a good reason to simplify `Component`.

For most compound components:

- you create them from many single dominoes
- while doing so, you need lots of local geometry editing operations
- but after the compound is finished, you usually do not keep editing it geometrically
- instead, you place the whole component somewhere and connect sockets with curves

So the generic `Component` API should focus on:

- parent/child tree
- `to_parent`
- `to_world()`
- `add_child()`
- `orient_abs()`
- `place_abs()`
- `move()`
- `rotate()`
- sockets

Those are universally useful.

By contrast, these are not universally useful:

- `place_snap()`
- `move_to_touch()`
- `rotate_to_touch()`
- a large anchor dictionary

Those are mostly geometry-authoring tools for specific subclasses.

## Why Anchors Should Not Be `Domino`-Only

`PileDomino` is the important counterexample.

Even if most compound components do not need snapping, some compound components still need to expose a geometric surface so that other things can snap to them.

Examples:

- a brick snapping to the side of a `PileDomino`
- a helper brick snapping to the top of a compound support structure
- a future component that intentionally behaves like one geometric support object

If anchors are only on `Domino`, then these cases become awkward immediately.

So the correct split is:

- not every `Component` has meaningful public anchors
- but some non-`Domino` components do

That means "has anchors" should be a capability, not the definition of `Component`.

## Recommended Model

Use three conceptual layers.

### 1. `Component`

Pure composition and transform node.

Responsibilities:

- tree structure
- local transform
- world transform
- sockets
- child management
- generic transform editing

Should not assume:

- it has a useful bounding box
- it has useful public anchors
- it supports touch operations

### 2. `AnchorableComponent`

A component that exposes named geometric anchors in its own local frame.

Responsibilities:

- define `anchor(name)`
- define a small set of public anchors
- support `place_snap()`

This can be:

- a subclass of `Component`
- or a mixin if you prefer that style

### 3. `TouchComponent`

A stricter capability for components that have enough geometric information to support contact-style operations like `move_to_touch()`.

Responsibilities:

- provide geometric shape / AABB / OBB / exact support data
- support one-DoF touch solves

Not every `AnchorableComponent` should automatically be a `TouchComponent`.

This distinction is useful because:

- anchor snapping is easy and deterministic
- move-until-touch is harder and more geometry-dependent

## Recommended Placement Of Methods

### Keep on `Component`

- `add_child()`
- `to_world()`
- `orient_abs()`
- `orient_like()`
- `orient()`
- `place_abs()`
- `move()`
- `rotate()`

These are generic transform operations.

One small refinement:

- `rotate()` on `Component` should probably be the simple version with no anchor, meaning rotate around the component's own local origin

because that is the most generic and least surprising transform edit.

If you want pivoted rotation, that is better treated as part of the anchorable editing capability.

### Move out of base `Component`

- `place_snap()`
- `move_to_touch()`
- `rotate_to_touch()`

These should live on capability-bearing classes only.

## Important Design Distinction

There are really two kinds of anchors:

- internal construction anchors
- public anchors

This distinction matters a lot.

### Internal anchors

Used while building a component from its children.

Examples:

- every face/corner anchor on every `Domino`
- temporary helper anchors used inside a gate implementation

These do not need to be visible outside the component.

### Public anchors

Exposed by the finished component to outside users.

Examples:

- the side of a `PileDomino`
- the top support surface of a compound support
- a carefully chosen alignment reference on a special mechanism

Not every compound component needs public anchors.

This solves your design tension cleanly:

- compounds may use lots of internal snapping while being constructed
- but only a few compounds expose a public geometric snapping interface afterward

## Recommended Public API Split

### Base `Component`

```python
class Component:
    def add_child(self, name: str, child: "Component") -> None: ...
    def to_world(self) -> Pose: ...
    def orient_abs(self, rotation_in_parent: np.ndarray) -> "Component": ...
    def orient_like(self, target: "Component") -> "Component": ...
    def orient(self, arg: "np.ndarray | Component") -> "Component": ...
    def place_abs(self, position_in_parent: np.ndarray) -> "Component": ...
    def move(self, delta_local: np.ndarray) -> "Component": ...
    def rotate(self, axis_local: np.ndarray, angle: float) -> "Component": ...
```

Note:

- `place_abs()` on base `Component` should not need anchors at all
- it should place the component origin
- this matches your preference for a simple generic transform API

### `AnchorableComponent`

```python
class AnchorableComponent(Component):
    def anchor(self, name: str) -> AnchorRef: ...
    def anchor_position_local(self, name: str) -> np.ndarray: ...
    def place_snap(self, anchor_name: str, target_anchor: AnchorRef) -> "AnchorableComponent": ...
    def rotate_about(self, anchor: str | np.ndarray | AnchorRef, axis_local: np.ndarray, angle: float) -> "AnchorableComponent": ...
```

### `TouchComponent`

```python
class TouchComponent(AnchorableComponent):
    def move_to_touch(self, axis: np.ndarray, target: "Component") -> "TouchComponent": ...
    def rotate_to_touch(self, anchor: str | np.ndarray | AnchorRef, axis: np.ndarray, target: "Component") -> "TouchComponent": ...
```

You may later decide that `rotate_to_touch()` is not worth supporting. That is fine. The important thing is not to force it into the base class.

## What `Domino` Should Be

`Domino` should be the main fully geometric primitive.

It should:

- inherit from `TouchComponent`
- provide a rich anchor set
- support snapping
- support contact-style moves

This fits your real authoring pattern:

- build structures by adding dominoes
- repeatedly orient/place/snap/move them during construction

`Domino` is exactly the class that benefits from the full 3D-software-like editing API.

## What `PileDomino` Should Be

`PileDomino` should be a compound component with selected public anchors.

Internally:

- it is built from several `Domino` children using the full anchor/snap API

Externally:

- it may expose a much smaller anchor interface

For example, `PileDomino` might expose:

- `x-`, `x+`, `y-`, `y+`, `z-`, `z+`
- `center`

but not every internal child anchor.

That gives exactly what you need:

- easy internal construction
- simple external snapping target

## How Compound Components Should Expose Anchors

Do not expose all child anchors automatically.

That would leak implementation details and make refactoring hard.

Instead, compound components should explicitly define public anchors in one of two ways.

### Option A: Anchor aliasing

Map a public anchor to a child anchor.

Example idea:

```python
self.public_anchors["x+"] = AnchorRef(self.children["top_brick"], "x+")
```

This is good when a component's outer reference point is naturally one exact child anchor.

### Option B: Anchor by derived geometry

Compute the public anchor from the compound's own derived bounding geometry.

Example:

- `PileDomino` side center
- `PileDomino` top center

This is better when the component should behave like one geometric block from the outside.

My recommendation:

- allow both
- use aliasing first because it is simpler
- use derived geometry when aliasing would expose implementation too much

## Recommended Rule For Sockets

Sockets should stay on base `Component`.

Reason:

- sockets are logical composition interfaces
- anchors are geometric editing interfaces

A component may have:

- sockets but no public anchors
- public anchors but no sockets
- both

This separation is healthy.

Examples:

- `ConditionGate` probably exposes sockets and maybe no public anchors
- `PileDomino` may expose public anchors and maybe no sockets
- `Domino` may expose anchors and maybe no sockets unless you intentionally give it some

## Recommended Rule For `place_abs`

If you simplify `Component`, then `place_abs()` should not take an anchor name.

It should mean:

- place the component origin at a parent-frame position

So the base API becomes:

```python
component.place_abs(np.array([x, y, z]))
component.orient_abs(Domino.standing())
component.move(np.array([dx, dy, dz]))
component.rotate(axis_local, angle)
```

Then snapping belongs to anchorable things:

```python
domino.place_snap("z-", pile.anchor("z+"))
```

This is a cleaner separation than having anchor-aware `place_abs()` in the base class.

## Recommended Rule For `rotate`

For base `Component`, keep only simple origin-based rotation:

```python
component.rotate(axis_local, angle)
```

If you need pivot rotation:

- that belongs on `AnchorableComponent`

for example:

```python
domino.rotate_about("x+y-z-", axis_local, angle)
domino.rotate_about(pile.anchor("x+"), axis_local, angle)
```

This is consistent with your observation that anchor-aware editing is mainly a primitive authoring operation.

## Should `move_to_touch` Exist On Compound Components?

Usually no.

There are two reasons:

- geometric meaning is often ambiguous for a compound with holes and protrusions
- exact box-box or shape-shape touching across a whole subtree is much more expensive and much less intuitive than on a primitive brick

So I recommend:

- `move_to_touch()` primarily for `Domino`
- maybe for a few special compound components that intentionally expose a simple outer hull
- not for arbitrary compounds by default

This is a good place to be opinionated.

## Practical Rule Of Thumb

Use this hierarchy:

- every object is a `Component`
- only some objects are `AnchorableComponent`
- only very few objects are `TouchComponent`

In your project, that probably means:

- `Component`: base tree node
- `Domino`: `TouchComponent`
- `PileDomino`: `AnchorableComponent`
- most other compound gates/triggers: plain `Component` with sockets only

## Example Of How Authoring Would Feel

### Building `PileDomino`

Internally:

```python
self.add_child("0", Domino().orient(Domino.lying()))
self.add_child("1", Domino().orient(Domino.lying()).place_snap("x+", self.children["0"].anchor("x-")))
self.add_child("2", Domino().orient(Domino.lying()).place_snap("x+", self.children["1"].anchor("x-")))
```

Externally:

```python
pile = PileDomino(3).place_abs(np.array([0.2, 0.0, 0.0]))
Domino().orient(Domino.standing()).place_snap("x-", pile.anchor("x+"))
```

The outside code does not need to know which child in `PileDomino` was used to define its public `x+` anchor.

### Building a gate-like component

```python
gate = ConditionGate().place_abs(np.array([1.0, 0.0, 0.0]))
root.add_child("gate", gate)
root.connect(...)
```

No need for public anchors if sockets are enough.

## Concrete Recommendation

I recommend the following design decisions.

### Base `Component`

- keep `parent`, `children`, `to_parent`, `to_world`
- keep sockets
- keep simple non-anchor transform editing
- remove anchor dictionary from the conceptual base API
- remove `place_snap`, `move_to_touch`, `rotate_to_touch` from base class

### `AnchorableComponent`

- add `anchor()`
- add `place_snap()`
- add `rotate_about()`
- optionally add `public_anchors`

### `TouchComponent`

- add `move_to_touch()`
- maybe add `rotate_to_touch()` later only if needed

### `Domino`

- full anchors
- full snap support
- touch support

### `PileDomino`

- built from internal domino snapping
- expose a small public anchor set
- probably no touch support at first

### Most other compound components

- sockets yes
- public anchors usually no
- touch support no

## Final Conclusion

Your instinct is mostly right:

- anchor-heavy editing should not dominate the base `Component` API
- it is primarily a primitive authoring tool

But the correct endpoint is not "anchors belong only to `Domino`".

The better design is:

- `Component` for generic transforms and sockets
- `AnchorableComponent` for geometric snapping
- `TouchComponent` for one-DoF contact solves

That keeps the common case simple while still covering special compound components like `PileDomino` cleanly.
