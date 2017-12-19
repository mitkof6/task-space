#ifndef INVERSE_DYNAMICS_MODEL_H
#define INVERSE_DYNAMICS_MODEL_H

/**
 * \file This file contains the implementation for the underlying representation
 * of the system's equations of motion.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

    /**
     * This is an abstract class that wraps the model's equations of motion
     * using the following convention
     *
     * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$
     *
     * where \f$ M \f$ is the inertia mass matrix, \f$ q, \dot{q}, \ddot{q} \f$
     * are the generalized coordinates and their derivatives respectively, \f$ f
     * \f$ are the forces that act on the model (e.g. gravity, Coriolis,
     * ligaments, etc.), \f$ \Phi \f$ is the constraint Jacobian matrix, \f$
     * \lambda \f$ are the Lagrangian multipliers and \f$ \tau \f$ are the
     * acting generalized forces.
     */
    class InverseDyanmicsModel : public ModelComponent {
	OpenSim_DECLARE_ABSTRACT_OBJECT(InverseDyanmicsModel, ModelComponent);
    public:

    };

}

#endif
