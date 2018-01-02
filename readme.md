Description
---

This project implements constraint and task space projection for OpenSim v4.0.
The purpose is to design well documented, very simple and light-weighted API
where the user can plan a movement in task space and perform simulations in a
mixed dynamics manner. A mixed dynamics scheme is a combination of an inverse
dynamics controller (task space equations of motion) that feds a forward
dynamics module.

The underlying task space controller supports task prioritization [1] and
constraint modeling [2-3]. As the constraints are implicitly accounted in the
inverse dynamics model this implementation is able to simulate accurately
models that use constraints. Examples include: absolute (Cartesian)
coordinates, closed kinematics chains and complex joint configurations (e.g.
shoulder, knee) [4].

Examples of using this API can be found in the relevant example folder.

- ExampleTaskBasedControl.cpp: a minimum example of controlling a single body
  in task space.

- ExampleAbsoluteCoordinates.cpp: a two body model built using absolute
  coordinates and constraints for modeling the joint restrictions, with a
  position planning controller (task space) for the end-effector body.

- ExampleClosedKinematicChain.cpp: three bodies are arranged in a closed
  kinematic chain topology and the planning is performed by defining the
  orientation of one of the bodies in task space.

- ExampleArm26.cpp: a musculoskeletal model that is controlled by multiple
  prioritized tasks where the muscle excitation that track the task goals are
  estimated using Task Space Computed Muscle Control [5].

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

- Test under-actuation

- Task Space Dynamic Inverse Kinematics [5]

- calcTotalForces() in InverseDynamics.cpp produces strage runtime erros on
  Arch-Linux, while on Windows this function works properly

Acknowledgment
---

[5] Dimitar Stanev and Konstantinos Moustakas, “Simulation of Constrained
Musculoskeletal Systems in Task Space”, IEEE Transaction on Biomedical
Engineering, vol. PP, no. 99 pp. 1-12, 2017, doi: 10.1109/TBME.2017.2764630

[SimTK Project](https://simtk.org/projects/task-space)
