#ifndef CONSTRAINT_PROJECTION_H
#define CONSTRAINT_PROJECTION_H

/**
 * \file This file contains the implementation of the constraint projection
 * models. TODO
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /**
     *
     */
    class ConstraintModel : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(ConstraintModel, ModelComponent);
    public:
        /**
         * Constrained inertia mass matrix inverse.
         */
        virtual SimTK::Matrix McInv(const SimTK::State& s) const = 0;
        /**
         * Constraint bias term.
         */
        virtual SimTK::Vector bc(const SimTK::State& s) const = 0;
        /**
         * Constraint null space matrix.
         */
        virtual SimTK::Matrix NcT(const SimTK::State& s) const = 0;
    protected:
        /**
         * Constraint Jacobian transpose matrix
         */
        virtual SimTK::Matrix JcT(const SimTK::State& s) const = 0;
        /**
         * Constraint Jacobian generalized inverse transposed.
         */
        virtual SimTK::Matrix JcBarT(const SimTK::State& s) const = 0;
    };
    /**
     *
     */
    class UnconstraintModel : public Constraint {
        OpenSim_DECLARE_CONCRETE_OBJECT(UnconstraintModel, ModelComponent);
    public:
        /**
         * Constrained inertia mass matrix inverse.
         */
        SimTK::Matrix McInv(const SimTK::State& s) const override;
        /**
         * Constraint bias term.
         */
        SimTK::Vector bc(const SimTK::State& s) const override;
        /**
         * Constraint null space matrix.
         */
        SimTK::Matrix NcT(const SimTK::State& s) const override;
    protected:
        /**
         * Constraint Jacobian transpose matrix
         */
        SimTK::Matrix JcT(const SimTK::State& s) const override;
        /**
         * Constraint Jacobian generalized inverse transposed.
         */
        SimTK::Matrix JcBarT(const SimTK::State& s) const override;
    };

}

#endif
