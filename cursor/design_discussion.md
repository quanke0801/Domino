now go over my previous project in `Domino/`, understand its structure, and help me design the new Mujoco-based project structure.

i like (and would like to keep) the `Component` design, where:
- The simplest component is just 1 single brick.
- Other components (e.g. line of bricks) is composed of multiple other components (e.g. n single bricks).
- Each component has input and output "sockets", so that an output socket of component A can be connected to an input socket of component B (with a curve component), and saves the user from manually computing the coordinates to place bricks to connect them.

Regarding the component/scene definition, previously it was quite tedious to manually compute the initial state of each brick/port to form a structure. e.g. The `Crossing` component contains bricks that are slanted (edge on one side touches ground, edge on the other side leans against another brick), and i had to manually use trigonometric functions to compute their initial position and orientations. it would be fantastic if there could be easier ways to do this:
- Running a sub-simulation loop to let them "rest" in the stable pose sounds fun, but is also complicated, inefficient, and cannot solve all cases, and yet i still need to manually put it in a correct initial pose for the sub-simulation to converge in the desired pose.
- probably need api to obtain the bounding box of a component, so that we don't need to explicitly compute how to stack another brick on top of an existing component.
- maybe an api to set the position of a specific point/edge/surface of a brick could be nice. e.g. if i want to put a brick sideways on top of a pile stack, bbox gives me the pile's height, but i want to set this height to the new brick directly, instead of passing something like "height + BRICK_WIDTH/2".
- maybe an api to set only the x and y coord, and let it drop down until it hit something, could also be useful.
- still no idea how to make it easier to initialize a brick that is leaning on another one.

Since mujoco has good support on visual rendering and materials, i would need the design to also support setting and storing the material properties. but preferably this should be completely separate from Components because the latter is purely body data definition. the material data itself should also support flexible settings, such as:
- give each block a random color from an enum set.
- assign a unique color for all bricks in a specific component, so that in the video we can easily tell which part is which component.
- override the texture of a single brick (the starting brick) with a fancy one, so that people know where to look.

Regarding the xml creation, the process requires body info, material info, and also engine setting info. preferably this should also be separate from component and material.
