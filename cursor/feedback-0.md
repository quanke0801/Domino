- i still need the component tree. should i keep only the relative `from_parent` transformation in each component? this is the no-duplicate and single-source-of-truth data storage, but it would bring complications when we need to create/align a component with another sibling component under the same parent (see below).
- i want the component creation to be concise. e.g. a crossing component can consist ~10 single domino bricks, and i need at least 10 lines of code to manually create them one by one (there's no saving in this, because it just has this level of logical complexity); however i would like to reduce the complexity of each creation, preferably only 1 line of code.
- i want the creation of each domino/component to be flexible e.g.:
    - put a standing domino on top of a lying domino, and align their xy centers. maybe the standing domino is rotated 90 degrees around z axis so that these 2 are sort of perpendicular.
    - put a domino sideways on ground.
    - put a standing domino on ground at position xyz and slide it towards x- direction until it touches another component. or in other words, create a standing domino on ground and align its side face with another component's some face/edge.
- you probably get the idea - i want to be able to build dominos relative to one another, a.k.a grow the structure, instead of pre-computing the center coordinate and rotation for each single one. what could be a unified way to design the Component class and APIs to support these (and maybe expandable)? i don't like creating a separate method for all the specific alignments in an ad-hoc way.
- it would also be nice if some common anchor points can be accessed in names rather than coordinates, e.g. TDLRFB for the 6 surfaces etc.

help me design, and output to `cursor_design-1.md` in this same folder.
