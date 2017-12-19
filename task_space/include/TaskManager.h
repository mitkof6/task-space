#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

/**
 * \file This file contains the implementation of task manager ...
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <memory>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    class TaskPriorityGraph;
    class ConstraintModel;

    class TaskManager : public ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(TaskManager, ModelComponent);
    public:
	TaskManager(TaskPriorityGraph* graph, ConstraintModel* constraintModel);
	SimTK::Vector calcTaskTorques(const SimTK::State& s);
    private:
	TaskPriorityGraph* taskPriorityGraph;
	ConstraintModel* constraintModel;
    };
}

#endif
