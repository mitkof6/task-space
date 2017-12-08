#ifndef TASK_PRIORITY_GRAPH_H
#define TASK_PRIORITY_GRAPH_H

#include <list>

/**
 * \file This file contains the implementation of the task priority graph.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

namespace OpenSim {

    class KinematicTask;

    /**
     *
     */
    class TaskPriorityGraph {
    public:
	TaskPriorityGraph();
	/**
	 * Adds a task and updates the priority sorted graph based on the parent
	 * task.
	 */
	void addTask(KinematicTask* task, KinematicTask* parent);
    private:
	/** A list containing the sorted tasks in priority order [high->low] */
	std::list<std::pair<KinematicTask*, KinematicTask*> > prioritySortedGraph;
    };
}

#endif
