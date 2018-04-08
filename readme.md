Description
---

This project implements constraint and task space projection for OpenSim v4.0.
The purpose is to design well documented, very simple and light-weighted API
where the user can plan a movement in task space and perform simulations in a
mixed dynamics manner. A mixed dynamics scheme is a combination of an inverse
dynamics controller (equations of motion in task space) that feds a forward
dynamics module.

The underlying task space controller supports task prioritization [1] and
constraint modeling [2-3]. As the constraints are implicitly accounted in the
inverse dynamics model this implementation is able to simulate accurately
models that use constraints. Examples include: absolute (Cartesian)
coordinates, closed kinematics chains and complex joint configurations (e.g.
shoulder, knee) [4].

Examples can be found in the src/taskspace/examples folder. Results are stored
in the data/results dir. All models are located in the data/ dir.

- ExampleTaskBasedControl.cpp: a minimum working example of controlling a
  single body in task space

- ExampleAbsoluteCoordinates.cpp: a two body model built using absolute
  coordinates and constraints for modeling the joint restrictions, with a
  position planning controller (task space) of the end-effector body; a
  selection matrix has been used to model under-actuation 

- ExampleClosedKinematicChain.cpp: three bodies are arranged in a closed
  kinematic chain topology and the planning is performed by defining the
  orientation of one of the bodies in task space

- ExampleArm26.cpp: a musculoskeletal model that is controlled by multiple
  prioritized tasks where the muscle excitation that track the task goals are
  estimated using Task Space Computed Muscle Control [5]

References
---

[1] Sentis, L. (2007). Synthesis and Control of Whole-Body Behaviors in
    Humanoid Systems. Stanford University. Retrieved from
    http://dl.acm.org/citation.cfm?id=1354211

[2] De Sapio, V., & Park, J. (2010). Multitask Constrained Motion Control Using
    a Mass-Weighted Orthogonal Decomposition. Journal of Applied Mechanics,
    77(4), 1–9. https://doi.org/10.1115/1.4000907

[3] Aghili, F. (2005). A unified approach for inverse and direct dynamics of
    constrained multibody systems based on linear projection operator:
    Applications to control and simulation. IEEE Transactions on Robotics,
    21(5), 834–849. https://doi.org/10.1109/TRO.2005.851380

[4] Mistry, M., & Righetti, L. (2014). Operational Space Control of Constrained
    and Underactuated Systems. Robotics: Science and Systems VII.

Dependencies
---

[OpenSim tested version](https://github.com/mitkof6/opensim-core/tree/stable_2)

TODO
---

This project is a re-implementation of
[github](https://github.com/mitkof6/opensim-task-space) so some features are
still missing.

- Task Space Dynamic Inverse Kinematics [5]

- Implement test cases

Acknowledgment
---

[5] Dimitar Stanev and Konstantinos Moustakas, “Simulation of Constrained
Musculoskeletal Systems in Task Space”, IEEE Transactions on Biomedical
Engineering, vol. 65, no. 2, pp. 307-318, 2018, doi: 10.1109/TBME.2017.2764630

[SimTK Project](https://simtk.org/projects/task-space)

<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img
alt="Creative Commons License" style="border-width:0"
src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is
licensed under a <a rel="license"
href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution
4.0 International License</a>.
