# `Domino.move_to_touch()` Design

Goal:

- `self` is one moving `Domino`
- `target` is any `Component`
- move `self` along `axis_in_parent`
- stop at the earliest translation where further motion would start penetrating any leaf domino inside `target`
- mere touching should not stop the slide if the motion is tangent to that contact, e.g. sliding horizontally on top of another domino

This requires continuous box-box collision detection, not just a static overlap test after each step.

## Core Idea

Model the moving domino as one oriented box translated along a line:

- rotation is fixed
- only translation changes
- target dominoes are static oriented boxes

For each target leaf domino, solve the interval of motion distance `t` for which the two OBBs overlap. Then:

- take the earliest nonnegative entry time `t_enter`
- among all target leaf dominoes, the minimum valid `t_enter` is the distance we can move

This is a swept OBB-vs-OBB test with fixed orientation, which can be solved robustly using SAT over a 1D motion parameter.

## Why This Handles "slide on top" Correctly

Suppose the moving domino is already touching another domino from above, and we slide horizontally.

On the vertical separating axis:

- the two boxes are exactly touching
- but the relative velocity projected onto that axis is zero

So that axis does not limit the allowed horizontal motion. The slide only stops when some other axis becomes the first one that would create penetration.

That is exactly the behavior you want.

## Geometry Representation

Represent each domino leaf as an OBB:

- center `c` in world frame
- orthonormal local axes `u0`, `u1`, `u2` in world frame
- half extents `e = Domino.SIZE / 2`

For a `Domino`, these come from:

- `to_world().position`
- columns of `to_world().rotation`
- `Domino.SIZE / 2`

## Step 1: Collect Target Leaf Dominoes

Add a helper conceptually like:

```python
def iter_leaf_dominoes(component: Component) -> Iterator[Domino]:
    ...
```

Rules:

- if a node is a `Domino`, yield it
- otherwise DFS through `children`

You do not need compound-level anchors or hulls here. `move_to_touch()` should test against real leaf domino geometry.

## Step 2: Convert Motion Axis To World Frame

`axis_in_parent` is expressed in `self.parent` frame.

Normalize it:

```python
direction_in_parent = axis_in_parent / np.linalg.norm(axis_in_parent)
```

Convert to world:

```python
parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
direction = parent_to_world.rotation @ direction_in_parent
```

Now the moving center is:

```python
c_self(t) = c0 + t * direction
```

where `t >= 0` is the slide distance.

## Step 3: Swept OBB-vs-OBB Test

For one moving box `A` and one static box `B`, use SAT on the usual 15 candidate axes:

- the 3 axes of `A`
- the 3 axes of `B`
- the 9 pairwise cross products `Ai x Bj`

Because there is no relative rotation, these axes are constant during the sweep.

For a candidate axis `n`, let:

- `rA(n)` = projection radius of `A` on `n`
- `rB(n)` = projection radius of `B` on `n`
- `r = rA + rB`
- `s0 = dot(cB - cA0, n)`
- `v = dot(direction, n)`

The overlap condition at slide distance `t` is:

```text
|s0 - t * v| <= r
```

or equivalently:

```text
-r <= s0 - t * v <= r
```

This defines an interval of valid `t` for that axis.

### Case A: `abs(v) < eps`

Motion has no component along this axis.

- if `abs(s0) > r + eps`, the boxes are separated forever on this axis, so there is no collision for this pair
- otherwise this axis imposes no bound on `t`

### Case B: `abs(v) >= eps`

Solve the two boundary roots:

```text
t0 = (s0 - r) / v
t1 = (s0 + r) / v
```

Then reorder:

```text
t_enter_axis = min(t0, t1)
t_exit_axis = max(t0, t1)
```

Intersect this with the running collision interval:

```text
t_enter = max(t_enter, t_enter_axis)
t_exit = min(t_exit, t_exit_axis)
```

If at any point:

```text
t_enter > t_exit + eps
```

then this pair can never collide.

## Step 4: Pair Result

After processing all SAT axes:

- if no collision interval survives, this target leaf does not block the slide
- otherwise the first contact distance for this pair is `t_enter`

Use only pairs with:

- `t_exit >= -eps`

and clamp:

- `t_contact = max(t_enter, 0.0)`

Then the overall slide distance is:

```text
t_min = min(t_contact over all target leaf dominoes)
```

If no pair yields a collision interval:

- there is no blocker in that direction
- either do nothing special, or require a max search distance

## Important Practical Question: Infinite Slide

The current API:

```python
move_to_touch(axis_in_parent, target)
```

does not specify a max distance.

That means if there is no blocker ahead, the function has no natural finite answer.

So I strongly recommend one of these:

### Option A

Require a `max_distance` parameter.

```python
move_to_touch(axis_in_parent, target, max_distance)
```

Then:

- if a blocker is found before `max_distance`, move to first contact
- otherwise move exactly `max_distance`

### Option B

Raise if no blocker is found.

This is less convenient for authoring.

I recommend Option A. It is much more practical and still supports the "move until first touch" intent.

## Robustness Details

### 1. Skip degenerate cross-product axes

If `Ai x Bj` has norm below `eps`, skip it.

That happens when the two axes are nearly parallel.

### 2. Normalize every SAT axis before projection

Projection radii and center distances should be computed on a unit axis.

### 3. Use an epsilon consistently

Recommended uses:

- axis degeneracy threshold
- zero-velocity-on-axis test
- interval intersection tolerance
- touching-vs-penetrating classification

### 4. Assume the initial state is non-penetrating

`move_to_touch()` is much simpler and more predictable if the precondition is:

- at `t = 0`, `self` is not penetrating `target`

Touching is allowed.

If initial penetration is possible, define behavior explicitly:

- either raise
- or return zero motion

I recommend:

- if any pair is penetrating at `t = 0` by more than `eps`, raise `ValueError`

That will expose bad scene construction early.

## Efficiency

For one moving domino against `N` target leaf dominoes:

- narrow phase cost is `O(15N)`

That is already fine for typical domino scenes.

If needed later, add a broad phase.

## Simple Broad Phase

Before swept SAT, cheaply reject target leaves that can never be hit.

Useful tests:

### 1. Directional half-space test

If the target box is entirely behind the motion direction and already separated enough, it cannot be the first blocker.

### 2. Swept AABB test

Compute:

- current OBB AABB of `self`
- end AABB after `max_distance`
- union them into a swept AABB

Then reject target leaf dominoes whose AABBs do not intersect the swept AABB.

This is only possible if you adopt `max_distance`.

### 3. Lower-bound candidate distance

Project both box centers and radii onto the motion direction to get a quick lower bound. If it is already larger than the best known hit, skip the pair.

My recommendation:

- implement without broad phase first
- add a simple center-projection lower bound if scenes get large

## Suggested Helper Structure

Keep `move_to_touch()` small by splitting it into helpers.

Suggested helpers:

```python
def iter_leaf_dominoes(component: Component) -> list[Domino]:
    ...

def domino_obb_world(domino: Domino) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ...

def swept_obb_hit_distance(
        moving_center: np.ndarray,
        moving_axes: np.ndarray,
        moving_half_extents: np.ndarray,
        direction: np.ndarray,
        target_center: np.ndarray,
        target_axes: np.ndarray,
        target_half_extents: np.ndarray,
        eps: float = 1e-9,
) -> float | None:
    ...
```

Then `move_to_touch()` becomes:

1. normalize `axis_in_parent`
2. convert to world direction
3. build moving domino OBB
4. DFS target leaves
5. for each leaf, run `swept_obb_hit_distance(...)`
6. take minimum positive hit distance
7. update `self.to_parent.position += distance * direction_in_parent`

## Pseudocode

```python
def move_to_touch(self, axis_in_parent, target, max_distance = np.inf):
    direction_in_parent = normalize(axis_in_parent)
    parent_to_world = self.parent.to_world() if self.parent is not None else Pose()
    direction = parent_to_world.rotation @ direction_in_parent

    cA, UA, eA = domino_obb_world(self)

    best_distance = max_distance
    hit_found = False

    for leaf in iter_leaf_dominoes(target):
        cB, UB, eB = domino_obb_world(leaf)
        distance = swept_obb_hit_distance(cA, UA, eA, direction, cB, UB, eB)
        if distance is None:
            continue
        if distance < -eps:
            raise ValueError("initial penetration detected")
        if distance <= best_distance:
            best_distance = max(distance, 0.0)
            hit_found = True

    self.to_parent.set_position(
        self.to_parent.position + best_distance * direction_in_parent
    )
    return self
```

If you do not want `max_distance`, then replace the final move with:

- move by `best_distance` if `hit_found`
- otherwise raise

## Why Not Binary Search?

A tempting approach is:

- repeatedly move the box
- perform static OBB-vs-OBB intersection tests
- binary search for first contact

I do not recommend that here.

Reasons:

- slower
- less exact
- more sensitive to epsilon choices
- harder to reason about tangential contacts

Since the motion is a pure translation with fixed orientation, the analytic swept-SAT solution is both cleaner and more robust.

## Final Recommendation

Implement `move_to_touch()` as:

- DFS over target leaf dominoes
- exact swept OBB-vs-OBB SAT
- choose the earliest valid entry distance
- move only along the requested parent-frame axis
- treat touching as allowed unless continued motion along that axis would create penetration

One API addition is strongly recommended:

- add `max_distance`

Without it, the "no blocker ahead" case does not have a good finite answer.
