i have a project (i wrote ~7 years ago, when i'm not yet good at software design) in `Domino/`, that uses pybullet to do domino brick simulation, to mimic electric current and logical gates.

now i have a need for creating tutorial & demonstration videos with domino bricks simulation, to illustrate how logical gates works to people don't know CPU at all. i tested with that project a bit, and it is difficult to extend, and quite impossible to save image/video easily. and i'm planning to rewrite the project with mujoco.

you have showed me that its python binding works, its built-in gui works well, and it has renderer to dump image/video. i think i have almost all the info needed to switch tech stack to it. a few more questions before that:

- how is the stability (specifically with scattered and stacked boxes) and efficiency compared to bullet engine? when using pybullet, i have to sacrifice some efficiency to lower the step size, otherwise the simulation would jitter and break.
- how is mujoco renderer's capability to set scene visual properties? e.g. texture, roughness, scene lighting, reflection, etc.
- what are some other aspect i should consider w.r.t a physics simulation engine in order to better suit my purpose?
