# MuJoCo Domino Rewrite Design

## Goals

- Preserve the old project's strongest idea: composable `Component`s with input/output sockets.
- Make scene authoring much easier than the old PyBullet version, especially for placement, stacking, and connecting structures.
- Keep physics structure, material styling, and engine/render settings as separate concerns.
- Make MJCF/XML generation a compilation target of the Python API, not the primary authoring surface for users.
- Support two workflows equally well: interactive viewing and deterministic frame/video export.

## Non-Goals

- Reproduce the old PyBullet class layout one-to-one.
- Hide all MuJoCo concepts completely. Some MuJoCo terms should remain visible in engine-facing layers.
- Build a full CAD or constraint solver for arbitrary resting poses in v1.

## What To Preserve From `Domino/`

- A component tree where a simple component is one brick and larger components are compositions of smaller ones.
- Named sockets/ports so components can be connected without manually solving global coordinates.
- Reusable geometry generators such as lines, curves, piles, gates, and triggers.
- A simulation layer separate from scenario construction.
- A recording/export path for tutorial and demonstration output.

## Problems In The Old Design

- Physics creation happened while recursively creating components, which tightly coupled scene definition to the engine.
- Placement logic often required manual trigonometry and hand-derived offsets.
- Body identity was too tied to engine-specific runtime IDs.
- Material or visual styling was not a first-class design concern.
- Simulation control, component structure, and test/inspection concerns were mixed too closely.

## Core Design Principles

- `Component` is pure scene definition. It does not talk to MuJoCo directly.
- MJCF generation is a compilation step from a higher-level Python scene graph.
- Every independently moving rigid body maps to its own MuJoCo `<body>` with a `freejoint` when appropriate.
- Visual/material data is stored separately from physical/component structure.
- Placement helpers should operate on anchors, bounding boxes, faces, edges, and sockets instead of raw coordinates whenever possible.
- All runtime lookup should use stable names, not engine-generated ordering.

## Proposed Architecture

### 1. Component Layer

This is the user-facing authoring API for logical structures.

Main concepts:

- `Component`: abstract composable object with a local transform, child components, sockets, and metadata.
- `LeafComponent`: component that contributes one or more physical rigid bodies.
- `CompositeComponent`: component that contains named children and exposes promoted sockets.
- `Socket`: named attachment point with pose and direction information.
- `PlacementResult`: resolved geometry output of a component after layout evaluation.

Responsibilities:

- Define local structure.
- Expose sockets.
- Support hierarchical composition.
- Stay engine-agnostic.

Examples:

- `Brick`
- `LineOfBricks`
- `CurveOfBricks`
- `Pile`
- `Crossing`
- `AndGate`
- `OrGate`
- `Trigger`

### 2. Placement And Geometry Layer

This layer solves the pain of manual coordinate computation.

Main concepts:

- `Pose`: position plus orientation.
- `Bounds`: axis-aligned bounding box in local or world frame.
- `Anchor`: semantic point on geometry such as center, top face center, left edge midpoint, or a custom socket point.
- `PlacementOp`: helper operation that places one object relative to another.

Key APIs to support:

- `component.bounds()`
- `component.anchor("top")`
- `component.anchor("edge_left_top")`
- `place_on_top(a, b)`
- `place_next_to(a, b, gap=...)`
- `align(anchor_a, anchor_b)`
- `set_xy_then_drop(component, x, y, clearance=...)`
- `connect(socket_a, socket_b, with_component=CurveOfBricks(...))`

Important design choice:

- Placement helpers should be deterministic geometry utilities first.
- Limited physics-assisted settling can be added later as an optional preprocessing tool, not as the default authoring path.

### 3. Physics Specification Layer

This layer describes physical bodies without committing to XML syntax too early.

Main concepts:

- `BodySpec`: one independently moving rigid body.
- `GeomSpec`: box, plane, mesh, etc.
- `JointSpec`: freejoint or other joint types.
- `PhysicsDefaults`: shared mass, density, friction, restitution-like settings if used, damping, solver-related defaults.

Responsibilities:

- Convert resolved components into rigid-body specs.
- Keep body naming stable and traceable back to components.
- Avoid direct renderer/material concerns except physical surface parameters that matter to simulation.

For dominoes:

- Each brick should become one `BodySpec`.
- During MJCF export, each brick body becomes one top-level child under `<worldbody>` with its own `<freejoint/>`.

### 4. Visual And Material Layer

This layer is fully separate from component geometry and physics layout.

Main concepts:

- `MaterialSpec`
- `TextureSpec`
- `VisualStyle`
- `StyleRule`
- `Palette`

Responsibilities:

- Define color, texture, reflectance, shininess/specular, roughness-like appearance controls available in MuJoCo's material model, transparency, and other visual properties.
- Assign styles to bodies or geoms by stable names, tags, or component membership.

Required use cases:

- Give all bricks in one component the same color.
- Choose random colors from a predefined palette.
- Override one specific brick's appearance.
- Keep "what it is" separate from "how it looks in the video".

Suggested styling mechanism:

- Components may expose semantic tags such as `component_id`, `role=start_brick`, `role=gate_input`, `role=gate_output`.
- A style system maps tags or names to `MaterialSpec`s during export.

### 5. MJCF Export Layer

This layer converts high-level scene data into valid MuJoCo XML.

Main concepts:

- `MjcfSceneBuilder`
- `MjcfBodyWriter`
- `MjcfAssetWriter`
- `MjcfDefaultsWriter`
- `MjcfVisualWriter`

Input sources:

- resolved component/body specs
- material/style assignments
- engine/render settings

Output sections:

- `<option>`
- `<default>`
- `<asset>`
- `<visual>`
- `<worldbody>`
- optionally `<contact>`, `<keyframe>`, and other advanced sections later

Design rule:

- This layer owns XML formatting and MuJoCo-specific naming/references.
- No placement math should live here.

### 6. Runtime Layer

This layer loads the generated MJCF into MuJoCo and drives simulation.

Main concepts:

- `SimulationConfig`
- `SimulationSession`
- `ViewerRunner`
- `OfflineRenderer`
- `FrameExporter`

Responsibilities:

- compile XML into `MjModel`
- create `MjData`
- step simulation
- optionally launch built-in viewer
- optionally render offscreen frames
- expose stable name-to-id lookups for diagnostics and testing

Suggested split:

- `runtime/viewer.py` for interactive GUI preview
- `runtime/render.py` for offscreen rendering and PNG/video export
- `runtime/session.py` for common stepping logic

### 7. Inspection And Testing Layer

The old plugin idea is still worth keeping, but it should be more focused.

Main concepts:

- `Observer`
- `EventProbe`
- `MotionStopCondition`
- `SimulationRecorder`

Responsibilities:

- observe body motion by stable body names
- detect first movement or trigger ordering
- stop when the scene settles
- record data for debugging or validation

This should not depend on component internals once the scene is compiled.

## Suggested Package Structure

```text
MujocoDomino/
  components/
    base.py
    brick.py
    line.py
    curve.py
    pile.py
    gates.py
    triggers.py
  geometry/
    pose.py
    bounds.py
    anchors.py
    placement.py
    curves.py
  physics/
    specs.py
    defaults.py
    resolver.py
  visuals/
    materials.py
    textures.py
    styles.py
  mjcf/
    scene_builder.py
    writers.py
  runtime/
    session.py
    viewer.py
    render.py
  observe/
    recorder.py
    conditions.py
    probes.py
  examples/
    ...
```

This structure intentionally separates:

- authoring and composition
- geometric reasoning
- physics body specification
- visual styling
- MuJoCo XML generation
- simulation and rendering

## Recommended Data Flow

1. User creates a tree of `Component`s.
2. A layout resolver computes promoted sockets, resolved poses, anchors, and bounds.
3. The component tree emits `BodySpec`s and semantic tags.
4. A style resolver assigns `MaterialSpec`s to those bodies/geoms.
5. An MJCF builder generates XML from:
   - body specs
   - style/material specs
   - engine/render settings
6. Runtime loads the XML into MuJoCo for viewer or offscreen rendering.

## Component API Sketch

This is the intended feel, not a final signature:

```python
root = Component("demo")

line1 = LineOfBricks(count=20, spacing="touching")
line2 = CurveOfBricks(radius=0.25, angle=90)

root.add("line1", line1)
root.add("line2", line2)
root.connect(line1.output("out"), line2.input("in"))

start = line1.brick(0)
start.style_tags.add("start_brick")

line2.place_next_to(line1, gap=0.02)
```

## Placement API Recommendations

The placement system matters more than the XML wrapper itself. Prioritize these features early:

- Bounding box query in local and world space.
- Named anchors on all primitive pieces.
- Relative placement helpers for top, bottom, left, right, front, back.
- Socket promotion from child to parent components.
- Curve generation between sockets.
- Optional "drop to support" helper for simple stacking.

What should wait until later:

- General leaning-pose solver.
- Automatic stable-pose sub-simulation as a first-class layout method.
- Complex contact-aware packing.

For difficult leaning or crossing structures, a practical v1 approach is:

- build small domain-specific helper constructors for common patterns
- encode the geometric relationships once
- reuse them as components

That is much safer than trying to solve every leaning contact procedurally.

## Naming And Identity

Use stable hierarchical names everywhere.

Examples:

- `demo/line1/brick_000`
- `demo/gate_a/input_curve/brick_003`

These names should propagate through:

- component graph
- body specs
- MJCF names
- runtime lookups
- recorder/test probes

This avoids the old problem of depending on engine body order.

## Materials And Rendering Recommendations

Separate "style intent" from final MuJoCo material instances.

Example flow:

- component emits tags like `component=line1`, `role=start_brick`
- style resolver decides color/texture/material
- MJCF exporter creates or reuses named `<material>` and `<texture>` assets

Good defaults to include in v1:

- palette-based color assignment
- per-component uniform color
- per-role override
- configurable ground material
- scene lights and camera presets
- offscreen framebuffer size settings for renderer output

## MuJoCo-Specific Notes

- The `<body>` hierarchy is a kinematic tree, not a grouping tree for independent rigid bodies.
- If domino bricks move independently, each brick needs its own body and usually a `freejoint`.
- Nested `<body>` without joints means rigid attachment to the parent.
- Therefore, many composite components in Python will compile into many sibling top-level MuJoCo bodies under `<worldbody>`.

This is not a problem. It just means the Python component tree and the MJCF body tree do not need to mirror each other exactly.

## Most Important Architectural Decision

Do not make the Python `Component` tree equal to the MuJoCo `<body>` tree.

Instead:

- Python `Component` tree is for authoring, composition, sockets, and placement.
- MuJoCo `<body>` tree is for physical articulation and runtime simulation.

For this project, the compiler between those two layers is the key abstraction.

## Recommended First Milestones

1. Implement `Pose`, `Socket`, `Bounds`, and a minimal `Component` base class.
2. Implement `Brick`, `LineOfBricks`, and `CurveOfBricks`.
3. Implement a resolver that turns components into `BodySpec`s with stable names.
4. Implement MJCF export for plane, box geom, freejoint, materials, lights, and camera.
5. Implement runtime preview and offline PNG export.
6. Add styling rules and per-component color assignment.
7. Rebuild one old gate component end-to-end as a validation case.

## Final Recommendation

The rewrite should treat MuJoCo as a backend compiler target, not as the center of the user API.

If you keep:

- a pure component composition layer
- a strong placement/anchor API
- a separate style system
- a separate MJCF export layer
- a thin runtime/viewer/render layer

then you will preserve the best part of the old project while removing the PyBullet-specific coupling that made it hard to extend.
