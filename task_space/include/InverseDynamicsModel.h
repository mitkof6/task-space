#ifndef INVERSE_DYNAMICS_MODEL_H
#define INVERSE_DYNAMICS_MODEL_H

/**
 * \file This file contains the implementation for the underlying representation
 * of the system's equations of motion using the following convention
 *
 * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$
 *
 * $\f \Phi \ddot{q} = b_c \f$
 *
 * where \f$ M \f$ is the inertia mass matrix, \f$ q, \dot{q}, \ddot{q} \f$ are
 * the generalized coordinates and their derivatives respectively, \f$ f \f$ are
 * the forces that act on the model (e.g. gravity, Coriolis, ligaments, etc.),
 * \f$ \Phi \f$ is the constraint Jacobian matrix, \f$ \lambda \f$ are the
 * Lagrangian multipliers, \f$ \tau \f$ are the acting generalized forces and
 * \f$ b_c \f$ is the constraint bias term.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <simbody/SimTKcommon.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {
    /**
     * Calculates the inverse inertia mass matrix of the system.
     */
    SimTK::Matrix calcMInv(const SimTK::State& s, const OpenSim::Model& model);
    /**
     * Calculates the total forces that act on the model (\f$ f \f$). This
     * requires that the model is realized to Stage::Dynamics. Muscle forces
     * are ignored since they are the actuation (i.e. \f$ \tau = R f_m \f$
     * and not \f$ f \f$).
     */
    SimTK::Vector calcTotalForces(const SimTK::State& s,
				  const OpenSim::Model& model);
    /**
     * Calculates the constraint Jacobian matrix (\f$ \Phi \f$).
     */
    SimTK::Matrix calcConstraintJacobian(const SimTK::State& s,
					 const OpenSim::Model& model);
    /**
     * Calculate the constraint bias term \f$ b_c \f$.
     */
    SimTK::Vector calcConstraintBias(const SimTK::State& s,
				     const OpenSim::Model& model);
}

#endif
