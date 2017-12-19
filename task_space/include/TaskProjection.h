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
 * Calculates the prioritized task Jacobian transpose matrix.
 *
 * \f$ \J_{t|t-1} = N_{t-1}^T J_t^T \f$
 *
 * @param J is the task Jacobian
 * @param NT is the null space matrix of the higher priority task
 */
SimTK::Matrix calcJpT(const SimTK::Matrix& J, const SimTK::Matrix& NT);

/**
 * Calculates the prioritized task inertia mass matrix. If JpT = JtT then there
 * will be no prioritization.
 *
 * \f$ \Lambda_{t|t-1} = (J_t M^{-1} N_{t-1}^T J_t^T)^{-1} \f$
 *
 * @param J is the task Jacobian
 * @param MInv is the inverse system mass matrix
 * @param JpT is the prioritized task Jacobian
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
 * @param J is the task Jacobian
 * @param MInv is the inverse system mass matrix
 */
SimTK::Matrix calcJBarT(const SimTK::Matrix& Lambda, const SimTK::Matrix& J,
			const SimTK::Matrix& MInv);

/**
 * Calculates the prioritized null space transpose matrix of the task.
 *
 * \f$ N_{t|t-1}^T = (I - J_{t|t-1}^T  \bar{J}_{t|t-1}^T) * N_{t-1}^T \f$
 *
 * @param Jt is the task Jacobian
 * @param NT is the null space matrix of the higher priority task
 */
SimTK::Matrix calcNPT(const SimTK::Matrix& JpT, const SimTK::Matrix& JBarpT,
		      const SimTK::Matrix& NT);

/**
 * Calculates the required task force that achieves the task goal.
 *
 * \f$ f_t = \Lambda{t|t-1} (\ddot{x}_t + b_t) + \bar{J}_{t|t-1}^T \tau_p \f$
 *
 * where \f$ \tau_p = f - \tau_{t-1} \f$, \f$f\f$ are the forces acting on the
 * mode and \f$ \tau_{t-1} \f$ is the contribution of the higher priority tasks.
 *
 * @param xddot is the task acceleration goal
 * @param Lambdap is the prioritized task inertia mass matrix
 * @param bt is the task bias term (\f$ -\dot{J}_t \dot{q} \f$)
 * @param JBarpT is the prioritized generalized inverse of the task Jacobian
 * @param taup are the generalized forces that act on the model
 */
SimTK::Vector calcft(const SimTK::Vector& xddot, const SimTK::Matrix& Lambdap,
		     const SimTK::Vector& bt, const SimTK::Matrix& JBarpT,
		     const SimTK::Vector& taup);

#endif
