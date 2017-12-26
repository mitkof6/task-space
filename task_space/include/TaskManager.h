/**
 * \file This file contains the implementation of task manager, which collects
 * the prioritized tasks, the constraint model and applies constraint and task
 * projection to evaluate the generalized forces that track the task goals.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <map>
#include <memory>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Storage.h>

namespace OpenSim {
    class KinematicTask;
    class TaskPriorityGraph;
    class ConstraintModel;
    /**
     * \brief Collects the necessary components for computing the generalized
     * forces using constraint and task projection.
     */
    class TaskManager : public ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(TaskManager, ModelComponent);
    public:
	/**
	 * This object does not take ownership of the TaskPriorityGraph and the
	 * ConstraintModel. They must be owned by the model
	 * (e.g. model.addComponent()).
	 */
	TaskManager(TaskPriorityGraph* graph, ConstraintModel* constraintModel);
	/**
	 * Evaluates the total task forces.
	 *
	 * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t + N_{g*} \tau_0 \f$
	 */
	SimTK::Vector calcTaskTorques(const SimTK::State& s);
	/**
	 * Print internal analytics to a .sto file format.
	 *
	 * @param prefix is added to the file (useful for batch simulations).
	 * @param dir is the directory where the results will be stored.
	 */
	void printResults(std::string prefix, std::string dir);
    protected:
	/** Perform some additional initialization. */
	void extendInitStateFromProperties(SimTK::State& s) const override;
    private:
	/** Priority sorted task graph. */
	TaskPriorityGraph* taskPriorityGraph;
	/** The constraint model. */
	ConstraintModel* constraintModel;
	/** Temporary data used for evaluating the constraint forces. */
	struct TaskData {
	    SimTK::Matrix NaT;
	    SimTK::Vector taua;
	};
	std::map<KinematicTask*, TaskData> taskCache;
	/** Stores information to internal variables. */
	mutable Storage analytics;
	/** Add data to analytics. */
	void appendAnalytics(const SimTK::State& s,
			     const SimTK::Vector& taskForces,
			     const SimTK::Vector& nullspaceForces);
    };
}

#endif
