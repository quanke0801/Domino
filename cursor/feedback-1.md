- `.place()` is a nice way of unify api. but i don't like the "accept all constraints" design, because then i need to resolve the constraint transformation, which is hard even to determine whether the constraints are enough or overkill.
- i would like to start simple. make `place()` take exactly a 3dof positional constraint and a 3dof rotational constrait. the position can be defined by only coinciding anchor-to-anchor or xyz coords for now; and the rotation can be defined by only selecting from presets or copy from another component or rpy angles for now.
- i do want to support "move until touch", but this can create chaos: if "move until touch" and "rotate until touch" both exist, there won't be a single solution. any ideas on resolving this?

you can output to `cursor_design-2.md`.
