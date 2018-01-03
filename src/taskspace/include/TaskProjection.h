/**
 * @file TaskProjection.h
 *
 * \brief Utilities for performing task space projection. For more details
 * please refer to Section II(E).
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_PROJECTION_H
#define TASK_PROJECTION_H

#include <SimTKmath.h>

/**
 * Calculates the prioritized, transposed task Jacobian matrix.
 *
 * \f$ J_{t|t-1*}^T = N_{t-1*}^T J_t^T \f$
 *
 * @param NaT is the aggregate null space matrix of the higher priority tasks
 * @param JT is the transposed task Jacobian matrix
 */
SimTK::Matrix calcJpT(const SimTK::Matrix& NaT, const SimTK::Matrix& JT);
/**
 * Calculates the task inertia mass matrix. If JT = JpT then this calculates the
 * prioritized task inertia mass matrix.
 *
 * \f$ \Lambda_{t|t-1*} = (J_t M^{-1} J_{t|t-1*}^T)^{-1} \f$
 *
 * @param J is the task Jacobian matrix
 * @param MInv is the inverse system mass matrix
 * @param JT is the transposed task Jacobian matrix
 */
SimTK::Matrix calcLambda(const SimTK::Matrix& J, const SimTK::Matrix& MInv,
                         const SimTK::Matrix& JT);
/**
 * Calculates the dynamically consistent generalized inverse, transpose of the
 * task Jacobian matrix. If \f$ \Lambda = \Lambda_{t|t-1*} \f$ then \f$
 * \bar{J}_t^T \f$ will be the prioritized generalized inverse (\f$
 * \bar{J}_{t|t-1*}^T \f$).
 *
 * \f$ \bar{J}_{t|t-1*}^T = \Lambda_{t|t-1*} J_t M^{-1} \f$
 *
 * @param Lambda is the inertia mass matrix of the task
 * @param J is the task Jacobian matrix
 * @param MInv is the inverse system mass matrix
 */
SimTK::Matrix calcJBarT(const SimTK::Matrix& Lambda, const SimTK::Matrix& J,
                        const SimTK::Matrix& MInv);
/**
 * Calculates the task's transposed null space matrix. If JT = JpT and JBarT =
 * JBarpT then this is the prioritized null space of the task.
 *
 * \f$ N_{t|t-1*}^T = I - J_{t|t-1*}^T  \bar{J}_{t|t-1*}^T \f$
 */
SimTK::Matrix calcNtT(const SimTK::Matrix& JT, const SimTK::Matrix& JBarT);
/**
 * Calculates the aggregate prioritized, transposed null space matrix of the
 * task.
 *
 * \f$ N_{t*}^T = N_{t|t-1*}^T N_{t-1*}^T, \; N_{t|t-1*}^T = (I - J_{t|t-1*}^T
 * \bar{J}_{t|t-1*}^T) \f$
 *
 * @param NtT is the task's transposed null space matrix matrix
 * @param NaT is the aggregate null space matrix of the higher priority tasks
 */
SimTK::Matrix calcNpT(const SimTK::Matrix& NtT, const SimTK::Matrix& NaT);
/**
 * Calculates the task forces that achieves the task goals. If Lambda = Lambdap,
 * JBarT = JBarpT and tau = f - tau_{t-1*} then this calculates the prioritized
 * task forces.
 *
 * \f$ f_t = \Lambda{t|t-1*} (\ddot{x}_t + b_t) + \bar{J}_{t|t-1*}^T \tau
 * \f$
 *
 * where \f$ \tau = f - \tau_{t-1*} \f$, \f$f\f$ are the forces acting on the
 * model and \f$ \tau_{t-1*} \f$ is the force contribution of the higher
 * priority tasks.
 *
 * @param xddot are the task acceleration goals
 * @param Lambda is the task inertia mass matrix
 * @param bt is the task bias term (\f$ -\dot{J}_t \dot{q} \f$)
 * @param JBarT is the generalized inverse transpose of the task Jacobian matrix
 * @param tau are the generalized forces that act on the model
 */
SimTK::Vector calcFt(const SimTK::Matrix& Lambda, const SimTK::Vector& xddot,
                     const SimTK::Vector& bt, const SimTK::Matrix& JBarT,
                     const SimTK::Vector& tau);
/**
 * Calculates the joint space generalized force contribution of the task. If JT
 * = JpT then this is the prioritized generalized force contribution of the
 * task.
 *
 * \f$ \tau_t = J_{t|t-1*}^T f_t\f$
 *
 * @param JT is the transposed task Jacobian matrix
 * @param f are the corresponding task forces
 */
SimTK::Vector calcTau(const SimTK::Matrix& JT, const SimTK::Vector& f);

#endif
