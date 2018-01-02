/**
 * @file TaskPriorityGraph.h
 *
 * \brief Implements a priority sorted graph using lists as the underlying data
 * structure representation. The elements of the list contain a pointer to a
 * task (child) and a pointer to the task with higher priority (parent). The
 * list is automatically rearranged upon insertion of the task pairs.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_PRIORITY_GRAPH_H
#define TASK_PRIORITY_GRAPH_H

#include <list>
#include <stdexcept>
#include "KinematicTask.h"

namespace OpenSim {
    /** \brief Thrown when the task exists in the graph to avoid directed
     * cycles.
     */
    class TaskExistsInGraphException : public std::logic_error {
        using std::logic_error::logic_error;
    };
    /** \brief Thrown when the provided parent task does not exist in the
     * graph.
     */
    class ParentNotInGraphException : public std::logic_error {
        using std::logic_error::logic_error;
    };
    /** short definition */
    typedef
        std::list<std::pair<KinematicTask*, KinematicTask*> > ListChildParent;
    /**
     * \brief A priority sorted graph of task pairs (child, parent).
     *
     * A container that holds a list of pairs (child, parent) in a priority
     * sorted order [high, low]. The pair contains a reference to the task and
     * the associated parent task. The priority graph is constructed by
     * supplying the child-parent pairs and the appropriated checks are made.
     *
     * Example:
     *
     *             pelvis        priority = 1
     *            /      \
     *        femur      torso   priority = 2
     *          /
     *       tibia               priority = 3
     *
     * \code{.cpp}
     * TaskPriorityGraph graph;
     * auto pelvis = new SpatialTask("pelvis", Vec3(0));
     * graph.addTask(pelvis, NULL);
     * auto femur = new OrientationTask("femur", Vec3(0));
     * graph.addTask(femur, pelvis);
     * auto tibia = new OrientationTask("tibia", Vec3(0));
     * graph.addTask(tibia, femur);
     * auto torso = new OrientationTask("torso", Vec3(0));
     * graph.addTask(torso, pelvis);
     * \endcode
     *
     * prioritySortedGraph = [
     *     {pelvis, 0},
     *     {torso, pelvis},
     *     {femur, pelvis},
     *     {tibia, femur}]
     */
    class TaskPriorityGraph {
    public:
        /**
         * Adds a task and updates the priority sorted graph based on the parent
         * task. This object does not take ownership of the
         * KinematicTask(s). They must be owned by the model
         * (e.g. model.addComponent()).
         *
         * @param task the task to be inserted.
         *
         * @param parent the associated parent task (higher priority).
         *
         * \throws TaskExistsInGraphException if inserting a task that has been
         * already inserted.
         *
         * \throws ParentNotInGraphException if the parent task (!NULL) is not
         * in the graph.
         */
        void addTask(KinematicTask* task, KinematicTask* parent);
        /** Get reference to the priority sorted list */
        const ListChildParent& getPrioritySortedGraph() const;
        /** cout << graph << endl; */
        friend std::ostream& operator<<(std::ostream& os,
                                        const TaskPriorityGraph& g) {
            for (auto pair : g.prioritySortedGraph) {
                if (pair.second == NULL) {
                    os << "Prent: 0\n\tChild: " << *pair.first << std::endl;
                } else {
                    os << "Prent: " << *pair.second << "\n\tChild: "
                        << *pair.first << std::endl;
                }
            }
            return os;
        };
    private:
        /** A list containing the sorted tasks in priority order [high->low] */
        ListChildParent prioritySortedGraph;
    };
}

#endif
