/**
 * @file InverseDynamicsModel.h
 *
 * \brief A wrapper for evaluating the equations of motion using the following
 * convention. For more details please refer to Section II(B).
 *
 * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$
 *
 * \f$ \Phi \ddot{q} = b \f$
 *
 * where \f$ M \f$ is the inertia mass matrix, \f$ q, \dot{q}, \ddot{q} \f$ are
 * the generalized coordinates and their derivatives respectively, \f$ f \f$
 * are the forces that act on the model (e.g. gravity, Coriolis, ligaments,
 * etc.), \f$ \Phi \f$ is the constraint Jacobian matrix, \f$ \lambda \f$ are
 * the Lagrangian multipliers, \f$ \tau \f$ are the acting generalized forces
 * and \f$ b \f$ is the constraint bias term.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef INVERSE_DYNAMICS_MODEL_H
#define INVERSE_DYNAMICS_MODEL_H

#include "internal/TaskSpaceExports.h"
#include <OpenSim/Simulation/Model/Model.h>

 /**
  * Calculates the inertia mass matrix.
  */
TaskSpace_API SimTK::Matrix calcM(const SimTK::State& s,
                                  const OpenSim::Model& model);
/**
 * Calculates the inverse inertia mass matrix.
 */
TaskSpace_API SimTK::Matrix calcMInv(const SimTK::State& s,
                                     const OpenSim::Model& model);
/**
 * Calculates the total force that act on the model (\f$ f \f$). This requires
 * that the model is realized to Stage::Dynamics. A working model is used as
 * this method may be called by a controller during numerical integration and
 * all controllers of the working model are removed to avoid infinite loops.
 * The actuators of the working model are disabled since they are the actuation
 * (i.e. muscles \f$ \tau = R f_m \f$ and not \f$ f \f$). Call this method only
 * from objects that are derived from OpenSim::Controller and never from
 * objects that are derived from OpenSim::Force.
 */
TaskSpace_API SimTK::Vector calcTotalGeneralizedForces(const SimTK::State& s,
                                                       const OpenSim::Model& model);
/**
 * Calculates the constraint Jacobian matrix (\f$ \Phi \f$).
 */
TaskSpace_API SimTK::Matrix calcConstraintJacobian(const SimTK::State& s,
                                                   const OpenSim::Model& model);
/**
 * Calculate the constraint bias term \f$ b = \Phi \ddot{q} \f$.
 */
TaskSpace_API SimTK::Vector calcConstraintBias(const SimTK::State& s,
                                               const OpenSim::Model& model);

#endif
