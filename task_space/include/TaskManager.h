#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

/**
 * \file This file contains the implementation of task manager which calculates
 * the joint space generalized forces that track the task goals.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    class TaskPriorityGraph;

    class TaskManager : public ModelComponent {
	OpenSim_DECLARE_ABSTRACT_OBJECT(TaskManager, ModelComponent);
    public:
	TaskManager(TaskPriorityGraph* graph);
	SimTK::Vector calcTaskTorques(const SimTK::State& s);
    private:
	TaskPriorityGraph* taskPriorityGraph;
	/**
	 * Calculates the prioritized task inertia mass matrix.
	 *
	 * \f$ \Lambda_{t|t-1} = (J_t M^{-1} N_{t-1}^T J_t^T) \f$
	 *
	 * @param Jt task Jacobian
	 * @param MInv inverse system mass matrix
	 * @param NT the null space matrix of the higher priority task
	 */
	SimTK::Matrix calcLambda(const SimTK::Matrix& Jt,
				 const SimTK::Matrix& MInv,
				 const SimTK::Matrix& NT);
    };

}

#endif
