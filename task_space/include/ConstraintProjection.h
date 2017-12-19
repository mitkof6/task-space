#ifndef CONSTRAINT_PROJECTION_H
#define CONSTRAINT_PROJECTION_H

/**
 * \file This file contains the various implementations of the constrained
 * dynamics.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /**
     * An abstract class.
     */
    class ConstraintModel : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(ConstraintModel, ModelComponent);
    public:
        /**
         * Calculates the inverse constraint inertia mass matrix.
         */
        virtual SimTK::Matrix McInv(const SimTK::State& s) const = 0;
        /**
         * Calculates the constraint bias term.
         */
        virtual SimTK::Vector bc(const SimTK::State& s) const = 0;
        /**
         * Calculates the constraint null space matrix.
         */
        virtual SimTK::Matrix NcT(const SimTK::State& s) const = 0;
    };
    /**
     * This model assumes that there are not constraints.
     *
     * \f$ M \ddot{q} + f = \tau \f$
     *
     * \f$ N_c^T = 1, \; b_c = 0 \f$
     */
    class UnconstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(UnconstraintModel, ConstraintModel);
    public:
        /** \f$ M^{-1} \f$  */
        SimTK::Matrix McInv(const SimTK::State& s) const override;
        /** \f$ b_c = 0 \f$ */
        SimTK::Vector bc(const SimTK::State& s) const override;
        /**  \f$ N_c^T = 1 \f$ */
        SimTK::Matrix NcT(const SimTK::State& s) const override;
    };
    /**
     * This model uses the inertia weighted generalized inverse of the
     * constraint Jacobian as adopted by DeSaptio et al. to derive the
     * constrained representation of the equations of motion.
     *
     * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$                        (1)
     *
     * \f$ \Phi \ddot{q} = b \f$                                             (2)
     *
     * @see InverseDynamicsModel.h
     *
     * The goal is to decouple constraint and applied forces through coordinate
     * projection. Multiply Eq. (1) from the left by \Phi M^{-1}, making use of
     * Eq. (2) and solve for \f$ \lambda \f$
     *
     * \f$ b + \Phi M^{-1} f + \Phi M^{-1} \Phi^T \lambda = \Phi M^{-1} \tau \f$
     *
     * \f$ \lambda = \bar{Phi}^T (\tau - f) - \Lambda_c b, ;\ \Lambda_c = (\Phi
     * M^{-1} \Phi^T)^{-1}, \; \bar{Phi}^T = \Lambda_c \Phi M^{-1} \f$       (3)
     *
     * Then combine Eqs. (1) and (3)
     *
     * \f$ M \ddot{q} + f^{\perp} + bc = \tau^{\perp} \f$
     *
     * \f$ b_c = - \Phi^T \Lambda_c b \f$
     *
     * \f$ f^{\perp} = N_c^T f, \; \tau^{\perp} = N_c^T \tau, \; N_c^T = 1 -
     * \Phi^T \bar{\Phi}^T \f$
     */
    class DeSapioModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(DeSapioModel, ConstraintModel);
    public:
        /** \f$ M^{-1} \f$  */
        SimTK::Matrix McInv(const SimTK::State& s) const override;
        /** \f$ b_c = -\Phi^T \Lambda_c b \f$ */
        SimTK::Vector bc(const SimTK::State& s) const override;
        /** N_c^T = 1 - \Phi^T \bar{\Phi}^T \f$ */
        SimTK::Matrix NcT(const SimTK::State& s) const override;
    private:
        /**
         * Calculates the constraint inertia mass matrix.
         *
         * \f$ \Lambda_c = (\Phi M^{-1} \Phi^T)^{-1} \f$
         */
        SimTK::Matrix Lambdac(const SimTK::State& s) const;
        /**
         * Calculates the constraint Jacobian matrix.
         */
        SimTK::Matrix Phi(const SimTK::State& s) const;
        /**
         * Calculates the generalized inverse transposed constraint Jacobian
         * matrix.
         *
         * \f$ \bar{\Phi}^T = \Lambda_c \Phi M^{-1} \f$
         */
        SimTK::Matrix PhiBarT(const SimTK::State& s) const;
    };

}

#endif
