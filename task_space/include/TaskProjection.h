#ifndef TASK_PROJECTION_H
#define TASK_PROJECTION_H

/**
 * \file This file contains the implementation of common utilities for task
 * space projection.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <simbody/SimTKcommon.h>

/**
 * Calculates the prioritized, transposed task Jacobian matrix.
 *
 * \f$ \J_{t|t-1} = N_{t-1}^T J_t^T \f$
 *
 * @param NT is the null space matrix of the higher priority task
 * @param JT is the transposed task Jacobian matrix
 */
SimTK::Matrix calcJpT(const SimTK::Matrix& NT, const SimTK::Matrix& JT);
/**
 * Calculates the prioritized task inertia mass matrix. If JpT = JtT then there
 * will be no prioritization.
 *
 * \f$ \Lambda_{t|t-1} = (J_t M^{-1} N_{t-1}^T J_t^T)^{-1} \f$
 *
 * @param J is the task Jacobian matrix
 * @param MInv is the inverse system mass matrix
 * @param JpT is the prioritized, transposed task Jacobian matrix
 */
SimTK::Matrix calcLambdap(const SimTK::Matrix& J, const SimTK::Matrix& MInv,
			  const SimTK::Matrix& JpT);
/**
 * Calculates the dynamically consistent generalized inverse, transpose of the
 * task Jacobian matrix. If \f$ \Lambda = \Lambda_{t|t-1} \f$ then \f$
 * \bar{J}_t^T \f$ will be the prioritized generalized inverse (\f$
 * \bar{J}_{t|t-1}^T \f$).
 *
 * \f$ \bar{J}^T = \Lambda J_t M^{-1} \f$
 *
 * @param Lmabda is the inertia mass matrix of the task
 * @param J is the task Jacobian matrix
 * @param MInv is the inverse system mass matrix
 */
SimTK::Matrix calcJBarT(const SimTK::Matrix& Lambda, const SimTK::Matrix& J,
			const SimTK::Matrix& MInv);
/**
 * Calculates the prioritized, transposed null space matrix of the task.
 *
 * \f$ N_{t|t-1}^T = (I - J_{t|t-1}^T  \bar{J}_{t|t-1}^T) * N_{t-1}^T \f$
 *
 * @param JpT is the prioritized, transposed task Jacobian matrix
 * @param NT is the aggregate null space matrix of the higher priority tasks
 */
SimTK::Matrix calcNPT(const SimTK::Matrix& JpT, const SimTK::Matrix& JBarpT,
		      const SimTK::Matrix& NT);
/**
 * Calculates the required task forces that achieves the task goals.
 *
 * \f$ f_t = \Lambda{t|t-1} (\ddot{x}_t + b_t) + \bar{J}_{t|t-1}^T \tau_p \f$
 *
 * where \f$ \tau_p = f - \tau_{t-1} \f$, \f$f\f$ are the forces acting on the
 * mode and \f$ \tau_{t-1} \f$ is the contribution of the higher priority tasks.
 *
 * @param xddot are the task acceleration goals
 * @param Lambdap is the prioritized task inertia mass matrix
 * @param bt is the task bias term (\f$ -\dot{J}_t \dot{q} \f$)
 * @param JBarpT is the prioritized generalized inverse transpose of the task
 * Jacobian matrix
 * @param taup are the generalized forces that act on the model
 */
SimTK::Vector calcFt(const SimTK::Matrix& Lambdap, const SimTK::Vector& xddot,
		     const SimTK::Vector& bt, const SimTK::Matrix& JBarpT,
		     const SimTK::Vector& taup);
/**
 * Calculates the joint space generalized forces from the task forces.
 *
 * \f$ \tau_t = J_t^T f_t\f$
 *
 * @param JT is the transposed task Jacobian matrix
 * @param f are the corresponding task forces
 */
SimTK::Vector calcTau(const SimTK::Matrix& JT, const SimTK::Vector& f);

#endif
