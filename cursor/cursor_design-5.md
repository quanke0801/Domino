# `Domino.rotate_to_touch()` Structure Design

Your current discomfort is a real design smell, and the root cause is this:

- the narrow-phase collision code is mixed with state mutation
- but conservative advancement is naturally an iterative planner operating on temporary states

So the fix is not "where should I put the loop?"

The fix is:

- separate immutable geometry evaluation from mutable component editing

Once you do that, the stepping-forward logic becomes straightforward and does not need backup/revert tricks.

## Short Recommendation

Split the problem into 3 layers:

### 1. Geometry snapshot layer

Pure value objects:

- `Pose`
- `OBB`
- maybe `RigidMotion` or `AngularSweep`

These should be cheap to create and easy to transform without touching `Component` / `Domino`.

### 2. Collision query layer

Pure functions or lightweight stateless classes:

- build SAT axes
- compute current separation
- compute one conservative safe step
- compute whether penetration already exists

This layer should never mutate a `Domino`.

### 3. Authoring API layer

Mutable component methods:

- `Domino.move_to_touch(...)`
- `Domino.rotate_to_touch(...)`

These methods should:

- create temporary geometry snapshots
- call the collision query layer
- only commit the final transform to `self` if a collision was found

That means:

- no backup/revert is needed
- no recursive `SAT.rotate()` calling itself is needed
- no stack overflow risk

## What Is Wrong With The Current Alternatives

### Option A: put the whole iterative loop inside `SAT.rotate()`

This is awkward because `SAT` currently owns:

- `obb1`
- `obb2`
- precomputed separation axes

But after each step:

- `obb1` changes
- the axis set should be recomputed
- the next step is really a new query, not a continuation of the same static SAT instance

So `SAT` is the wrong place for the whole loop.

`SAT` should answer a local geometric question, not own a stateful motion process.

### Option B: loop in `Domino.rotate_to_touch()` by mutating `self` each step

This works mechanically, but it pollutes the editing API with speculative state updates.

That leads to exactly the problem you described:

- you need backup/revert
- failure leaves temporary mutation concerns
- code becomes harder to reason about

So that is also not the right structure.

## Recommended Architecture

The best pattern is:

- `Domino.rotate_to_touch()` is a transaction-like method
- it asks a pure solver for the final safe angle
- if the solver returns an angle, apply one single final `self.rotate(...)`
- otherwise do nothing

So the iterative stepping happens, but only on temporary geometric snapshots.

## Core Design Principle

Do not iterate on `Domino`.

Iterate on:

- `Pose`
- `OBB`
- or a dedicated temporary `RotatingOBBState`

These are disposable values.

Then at the end:

- commit once to `Domino`

This is the cleanest possible design.

## Concrete Class / Function Split

## 1. Keep `OBB` as a value type

Add geometry-only transform helpers to `OBB`.

Recommended methods:

```python
class OBB:
    def translated(self, delta: np.ndarray) -> "OBB": ...
    def rotated(self, anchor: np.ndarray, rotation: np.ndarray) -> "OBB": ...
```

Important:

- these return new `OBB`s
- they do not mutate `self`

Then conservative advancement can update a temporary `OBB` each step without touching any `Domino`.

This is not "annoying extra rotation logic". This is exactly the right abstraction boundary.

If an object represents geometry, it should know how to return a transformed copy of itself.

## 2. Rename `SAT.rotate()` to a one-step bound query

Your instinct here is correct.

It should not be called `rotate()` if it does not actually carry out the full iterative sweep.

Recommended names:

- `conservative_rotation_step(...)`
- `rotation_step_bound(...)`
- `max_safe_rotation_step(...)`

Its job should be:

- given current `obb1`, `obb2`, pivot, and angular velocity direction
- compute a conservative upper bound on how much `obb1` may rotate before the next possible contact event

So it returns one step size only.

Example signature:

```python
def max_safe_rotation_step(
        obb1: OBB,
        obb2: OBB,
        anchor: np.ndarray,
        omega1: np.ndarray,
        eps: float = EPSILON,
) -> float | None:
    ...
```

This should be a pure function, or a `@staticmethod`, or a method on a short-lived query object.

I would prefer a pure function or static method.

## 3. Add a higher-level conservative advancement solver

This is the layer that owns the loop.

Example:

```python
def first_rotation_contact_angle(
        obb1: OBB,
        obb2_list: list[OBB],
        anchor: np.ndarray,
        axis: np.ndarray,
        max_angle: float = 2 * np.pi,
        eps: float = EPSILON,
        max_iters: int = 100,
) -> float | None:
    ...
```

Responsibilities:

- maintain current temporary `obb1`
- repeatedly ask every target OBB for a conservative safe step
- take the smallest step over all targets
- rotate the temporary `obb1`
- stop when separation is below tolerance
- stop when traveled angle exceeds `max_angle`
- return the first contact angle if found, otherwise `None`

This function is the natural place for the stepping-forward mechanism.

Not `SAT`.
Not `Domino`.

## Why This Is The Right Place For The Loop

Because conservative advancement is neither:

- a primitive static SAT query

nor:

- a scene-object editing operation

It is a motion solver.

So give it a motion-solver-level function.

## Suggested Module Structure

Your current `geometry/collision.py` is already close to this split. I would evolve it like this:

```text
geometry/
  collision.py
    class OBB
    def sat_axes(obb1, obb2)
    def separation_distance(obb1, obb2)
    def swept_translation_hit_distance(...)
    def max_safe_rotation_step(...)
    def first_rotation_contact_angle(...)
```

That keeps all temporary geometry logic in one place.

## Recommended Control Flow For `Domino.rotate_to_touch()`

`Domino.rotate_to_touch(...)` should be very small.

Conceptually:

1. collect target leaf dominoes
2. build `OBB` snapshots for all of them
3. build `self` OBB snapshot
4. convert anchor / axis to world frame
5. call `first_rotation_contact_angle(...)`
6. if angle found:
   - convert axis back to parent/local convention if needed
   - apply one final `self.rotate(...)`
7. otherwise:
   - leave `self` untouched

This gives you exactly the semantics you want:

- if no collision happens, no mutation
- if collision happens, one final mutation

## Example Shape

```python
def rotate_to_touch(self, anchor: np.ndarray, axis_in_parent: np.ndarray, target: Component) -> "Domino":
    target_obbs = [domino.obb_in_world() for domino in target.collect_dominoes()]
    self_obb = self.obb_in_world()

    parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
    axis_in_parent = axis_in_parent / np.linalg.norm(axis_in_parent)
    axis_in_world = parent_to_world.rotation @ axis_in_parent
    anchor_in_world = parent_to_world * anchor

    angle = first_rotation_contact_angle(self_obb, target_obbs, anchor_in_world, axis_in_world)
    if angle is not None:
        self.rotate(anchor, axis_in_parent, angle)
    return self
```

This is the right high-level API shape.

## Important Refinement: Anchor Type

You should decide clearly whether the `anchor` argument to `rotate_to_touch()` is:

- in self local frame
- in parent frame
- or in world frame

Given your existing `rotate()` method, the cleanest choice is:

- `anchor` is in self local frame, or an anchor name / `AnchorRef`

Then `rotate_to_touch()` can resolve it to world once at the start.

That keeps the authoring API consistent.

## Should `SAT` Be A Class At All?

Possibly not.

Your current `SAT` object is mostly:

- `obb1`
- `obb2`
- precomputed axes

That is useful for one static query, but once you move into conservative advancement, the "stateful SAT object" buys you very little, because each step has a new `obb1`.

You may find the code cleaner if you demote `SAT` into pure helper functions:

```python
def separation_axes(obb1: OBB, obb2: OBB) -> list[np.ndarray]: ...
def overlap_interval_along_axis(...): ...
def swept_translation_time(...): ...
def max_safe_rotation_step(...): ...
```

If you want to keep `SAT`, that is still fine, but then I would restrict it to:

- one static pair query object
- no iterative motion ownership

## Recommended Semantics For The One-Step Rotation Bound

Your current method:

- estimates max point speed
- computes global separation distance
- returns `distance / speed`

That is a good conservative advancement pattern.

So keep that concept, but rename and narrow its role:

- it returns only one admissible step for the current pair and current configuration

Then the higher-level loop handles:

- repeated stepping
- rebuilding current `obb1`
- checking termination

## Termination Logic

The motion solver should explicitly own these conditions:

### Contact found

If the current separation distance is below tolerance:

- return current accumulated angle

### Safe step too small

If the next conservative step is below tolerance:

- treat as contact

### No collision within full revolution

If accumulated angle exceeds `2 * np.pi`:

- return `None`

### Numerical stall

If loop count exceeds `max_iters`:

- either raise
- or return `None` with a warning

I recommend:

- return `None` and log warning for v1

## Multiple Targets

For `target` as a whole component:

- at each iteration, evaluate all target OBBs
- choose the minimum safe step across them

This is exactly analogous to your `move_to_touch()` design.

Do not create one big compound OBB.
Use the real leaf dominoes.

## Why No Backup/Revert Is Needed

Because the solver never mutates `self`.

The only mutable step is the very last one:

- `self.rotate(...)`

and that happens only if a final collision angle is found.

So the authoring object stays clean.

That is the main architectural win.

## The Missing Abstraction

If you want one sentence to summarize the design flaw:

You currently do not have an abstraction for "temporary motion state".

You have:

- static geometry (`OBB`)
- persistent scene objects (`Domino`)

but conservative advancement needs a third thing:

- temporary evolving geometry during a query

You do not need a heavyweight class for it, though. A loop over temporary `OBB` values is enough.

## Minimal Change Path

If you want the smallest refactor from where you are now:

### Keep:

- `Domino.obb_in_world()`
- `target.collect_dominoes()`
- `Domino.rotate(...)`
- `OBB`

### Change:

- add `OBB.rotated(...)`
- rename `SAT.rotate()` to something like `max_safe_rotation_step(...)`
- add one new function `first_rotation_contact_angle(...)`
- make `Domino.rotate_to_touch()` call that function and commit once

This is probably the best balance of cleanliness and minimal disruption.

## Final Recommendation

The stepping-forward mechanism should live in a dedicated motion-solver function in `geometry/collision.py`, not in `SAT.rotate()` and not by mutating `Domino` incrementally.

Use this structure:

- `OBB`: immutable geometry value with transform helpers
- `SAT` or helper functions: one-step local collision bounds
- `first_rotation_contact_angle(...)`: conservative advancement loop on temporary OBBs
- `Domino.rotate_to_touch(...)`: thin wrapper that resolves frames, calls solver, and commits once if successful

That structure is clean, robust, and fits your current codebase well.
