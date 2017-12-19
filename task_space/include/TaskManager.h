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
#include <simbody/SimTKcommon.h>

namespace OpenSim {
    class TaskPriorityGraph;

    class TaskManager {
    public:
	TaskManager(std::shared_ptr<TaskPriorityGraph> graph);
	SimTK::Vector calcTaskTorques(const SimTK::State& s);
    private:
	std::shared_ptr<TaskPriorityGraph> taskPriorityGraph;
    };
}

#endif
