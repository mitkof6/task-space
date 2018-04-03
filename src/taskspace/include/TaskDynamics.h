/**
 * @file TaskDynamics.h
 *
 * \brief Task dynamics combines task and constraint space projection to
 * evaluate the task force contribution accounting for the task prioritzation
 * and model constraints. For more details please refer to Section II(E-F).
 *
 * A priority sorted graph using lists as the underlying data structure
 * representation. The elements of the list contain a pointer to a task (child)
 * and a pointer to the task with higher priority (parent). The list is
 * automatically rearranged upon insertion of the task pairs.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

#include <list>
#include <stdexcept>
#include "internal/TaskSpaceExports.h"
#include "KinematicTask.h"
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    class ConstraintModel;
    /** \brief Thrown when the task exists in the graph to avoid directed
    * cycles.
    */
    class TaskSpace_API TaskExistsInGraphException : public std::logic_error {
        using std::logic_error::logic_error;
    };
    /** \brief Thrown when the provided parent task does not exist in the
    * graph.
    */
    class TaskSpace_API ParentNotInGraphException : public std::logic_error {
        using std::logic_error::logic_error;
    };
    /** short definition */
    typedef
        std::list<std::pair<KinematicTask*, KinematicTask*> > ListChildParent;
    /**
     * \brief Collects the necessary components for computing the applied
     * generalized forces using constraint and task space projection.
     *
     * For a more flexible implementation calcTaskDynamicsData() can be invoked
     * by any external procedure that will have access to the TaskDynamicsData
     * and define its own control law. The TaskDynamicsData can provide
     * additional information if required in future versions in order to to
     * support additional control strategies.
     *
     * Invoking calcTaskDynamicsData() returns a struct containing
     * pre-calculated variables for implementing different control laws. For
     * example we can choose to utilize the task forces and minimize the
     * nullspace forces or model under-actuated systems [1].
     *
     * Example 1 (control without nullspace forces \f$ \tau_0 = 0 \f$):
     *
     * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t \f$
     *
     * \code{.cpp}
     * auto data = taksDynamics->calcTaskDynamicsData();
     * Vector tau = data.tauTasks;
     * \endcode
     *
     * Example 2 (compensate gravity, Coriolis and constraint bias using the
     * nullspace):
     *
     * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t + N_{g*}^T (f + b_c) \f$
     *
     * \code{.cpp}
     * auto data = taskDynamics.calcTaskDynamicsData();
     * Vector tau = data.tauTasks + data.NgT * (data.f + data.bc);
     * \endcode
     *
     * Example 3 (utilize the nullspace in order to model under-actuated
     * systems when the system contains passive DoFs [1]):
     *
     * \f$ \tau = (I - N_{g*}^T [(I - B) N_{g*}^T]^+) \sum_{t=1}^g
     * J_{t|t-1*}^T f_t \f$
     *
     * \code{.cpp}
     * Matrx B; // diagonal with "1" for active DoFs and "0" for passive
     * auto data = taskDynamics.calcTaskDynamicsData();
     * auto A = (1 - B) * data.NgT;
     * Matrix AInv;
     * FactorSVD svd(A);
     * svd.inverse(AInv);
     * Vector tau = (1 - data.NgT * AInv) * data.tauTasks;
     * \endcode
     */
    class TaskSpace_API TaskDynamics : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskDynamics, ModelComponent);
    public:
        /**
         * This object does not take ownership of the ConstraintModel. They
         * must be owned by the model (e.g. model.addComponent()).
         */
        TaskDynamics(ConstraintModel* constraintModel, const SimTK::Matrix& S);
        /**
        * Adds a task and updates the priority sorted graph based on the parent
        * task. This object does not take ownership of the KinematicTask(s).
        * They must be owned by the model (e.g. model.addComponent()).
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
        /**
         * \brief Data used for implementing different control strategies.
         */
        struct TaskDynamicsData {
            /**
             * The total generalized forces of the tasks.
             *
             * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t \f$
             *
             * @see TaskProjection.h
             */
            SimTK::Vector tauTasks;
            /**
             * The total null space transpose.
             *
             * \f$ N_{g*}^T = \prod_{t=1}^g N^T_{t|t-1*} N^T_{i-1*}, \; N_{0*}
             * = N_c^T (constraints)\f$
             *
             * @see TaskProjection.h
             */
            SimTK::Matrix NgT;
            /**
             * Total acting forces.
             *
             * \f$ M \ddot{q} + f = \tau \f$
             *
             * @see InverseDynamicsModel.h
             */
            SimTK::Vector f;
            /**
             * The constraint bias term \f$ b_c \f$.
             *
             * @see ConstraintProjection.h
             */
            SimTK::Vector bc;
        };
        /**
         * Computes the TaskDynamicsData.
         */
        TaskDynamicsData calcTaskDynamicsData(const SimTK::State& s);
        /** cout << graph << endl; */
        friend std::ostream& operator<<(std::ostream& os,
                                        const TaskDynamics& g) {
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
        /** A reference to the constraint model. */
        ConstraintModel* constraintModel;
        /** A list containing the sorted tasks in priority order [high->low] */
        ListChildParent prioritySortedGraph;
        /** Selection matrix \f$ \tau^{'} = S \tau \f$ for under-actuation. */
        SimTK::Matrix S;
    };
}

#endif
