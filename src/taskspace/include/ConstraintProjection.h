/**
 * @file ConstraintProjection.h
 *
 * \brief The implementations of the constraint projection. For more details
 * please refer to Section II(C).
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef CONSTRAINT_PROJECTION_H
#define CONSTRAINT_PROJECTION_H

#include "internal/TaskSpaceExports.h"
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /**
     * \brief An abstract class for defining constraint models.
     */
    class TaskSpace_API ConstraintModel : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(ConstraintModel, ModelComponent);
    public:
        /** \brief Calculated by calcConstraintData(). */
        struct ConstraintData {
            /** Constraint inertia mass matrix inverse.*/
            SimTK::Matrix McInv;
            /** Constraint nullspace matrix. */
            SimTK::Matrix NcT;
            /** Constraint bias term. */
            SimTK::Vector bc;
        };
        /**
        * Calculates the constraint data. Note that in the future additional
        * ConstraintData may be provided.
        */
        virtual ConstraintData calcConstraintData(const SimTK::State& s) const = 0;
    };
    /**
     * \brief This model assumes that there are not constraints.
     *
     * \f$ M \ddot{q} + f = \tau \f$
     *
     * \f$ M_c^{-1} = M^{-1}, \; b_c = 0, \; N_c^T = 1 \f$
     */
    class TaskSpace_API UnconstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(UnconstraintModel, ConstraintModel);
    public:
        ConstraintData calcConstraintData(const SimTK::State& s) const override;
    };
    /**
     * \brief This model uses the inertia weighted generalized inverse of the
     * constraint Jacobian as adopted by DeSaptio et al. [1] to derive the
     * constrained representation of the equations of motion.
     *
     * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$                       (1)
     *
     * \f$ \Phi \ddot{q} = b \f$                                            (2)
     *
     * @see InverseDynamicsModel.h
     *
     * The goal is to decouple constraint and applied forces through coordinate
     * projection. Multiply Eq. (1) from the left by \f$ \Phi M^{-1} \f$,
     * making use of Eq. (2) and solve for \f$ \lambda \f$
     *
     * \f$ b + \Phi M^{-1} f + \Phi M^{-1} \Phi^T \lambda = \Phi M^{-1} \tau
     * \f$
     *
     * \f$ \lambda = \bar{Phi}^T (\tau - f) - \Lambda_c b, ;\ \Lambda_c = (\Phi
     * M^{-1} \Phi^T)^{-1}, \; \bar{Phi}^T = \Lambda_c \Phi M^{-1} \f$      (3)
     *
     * Then combine Eqs. (1) and (3)
     *
     * \f$ M \ddot{q} + f^{\perp} + bc = \tau^{\perp} \f$
     *
     * \f$ b_c = - \Phi^T \Lambda_c b \f$
     *
     * \f$ f^{\perp} = N_c^T f, \; \tau^{\perp} = N_c^T \tau, \; N_c^T = 1 -
     * \Phi^T \bar{\Phi}^T \f$
     *
     * [1] De Sapio, V., & Park, J. (2010). Multitask Constrained Motion
     * Control Using a Mass-Weighted Orthogonal Decomposition. Journal of
     * Applied Mechanics, 77(4), 1–9. https://doi.org/10.1115/1.4000907
     */
    class TaskSpace_API DeSapioModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(DeSapioModel, ConstraintModel);
    public:
        ConstraintData calcConstraintData(const SimTK::State& s) const override;
    };
    /**
     * \brief This model make uses of the Moore - Penrose pseudoinverse (MPP)
     * and the theory of linear projection operators to decouple the constraint
     * and applied forces based on Aghili [1].
     *
     * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$                       (1)
     *
     * \f$ \Phi \ddot{q} = b \f$                                            (2)
     *
     * \f$ N_c = N_c^T = I - \Phi^+ \Phi \f$                                (3)
     *
     * Reexpress Eqs. (1) and (2) using (3)
     *
     * \f$ N_c M \ddot{q} + N_c f = N_c \tau, \; N_c \Phi^T = 0 \f$
     *                                                                      (4)
     * \f$ \ddot{q}_{\parallel} = M (I - N_c) \ddot{q} = \Phi^+ b \f$
     *
     * Combining Eqs. (4) together we can derive the model
     *
     * \f$ M^{'} \ddot{q} + f_{perp} + b_c = \tau_{\perp} \f$
     *
     * \f$ M^{'} = M + N_c M - (N_c M)^T \f$
     *
     * \f$ f^{\perp} = N_c f, \; \tau^{\perp} = N_c \tau \f$
     *
     * \f$ b_c = -M \Phi^+ b \f$
     *
     * [1] Aghili, F. (2005). A unified approach for inverse and direct
     * dynamics of constrained multibody systems based on linear projection
     * operator: Applications to control and simulation. IEEE Transactions on
     * Robotics, 21(5), 834–849. https://doi.org/10.1109/TRO.2005.851380
     */
    class TaskSpace_API AghiliModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(AghiliModel, ConstraintModel);
    public:
        ConstraintData calcConstraintData(const SimTK::State& s) const override;
    };
}

#endif
