/**
 * \file Task dynamics combines task and constraint space projection to evaluate
 * the task force contribution accounting for the task prioritzation and model
 * constraints. For more details please refer to Section II(E-F).
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    class KinematicTask;
    class TaskPriorityGraph;
    class ConstraintModel;
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
     * Example 1 (control without nullspace forces \$f \tau_0 = 0 \f$):
     *
     * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t \f$
     *
     * auto data = taksDynamics->calcTaskDynamicsData();
     * Vector tau = data.tauTasks;
     *
     * Example 2 (compensate gravity, Coriolis and constraint bias using the
     * nullspace):
     *
     * \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t + N_{g*}^T (f + b_c) \f$
     *
     * auto data = taskDynamics.calcTaskDynamicsData();
     * Vector tau = data.tauTasks + data.NgT * (data.f + data.bc);
     *
     * Example 3 (utilize the nullspace in order to model under-actuated
     * systems when the system contains passive DoFs [1]):
     *
     * \f$ \tau = (I - N_{g*}^T [(I - B) N_{g*}^T]^+) \sum_{t=1}^g
     * J_{t|t-1*}^T f_t \f$
     *
     * Matrx B; // diagonal with "1" for active DoFs and "0" for passive
     * auto data = taskDynamics.calcTaskDynamicsData();
     * auto A = (1 - B) * data.NgT;
     * Matrix AInv;
     * FactorSVD svd(A);
     * svd.inverse(AInv);
     * Vector tau = (1 - data.NgT * AInv) * data.tauTasks;
     */
    class TaskDynamics : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskDynamics, ModelComponent);
    public:
        /**
         * This object does not take ownership of the TaskPriorityGraph and the
         * ConstraintModel. They must be owned by the model
         * (e.g. model.addComponent()).
         */
        TaskDynamics(TaskPriorityGraph* graph, ConstraintModel* constraintModel);
        /**
         * Data used for implementing different control strategies.
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
             * \f$ N_{g*}^T = \prod_{t=1}^g N^T_{t|t-1*} N^T_{i-1*}, \; N_{0*} =
             * N_c^T (constraints)\f$
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
    private:
        /** A reference to the priority sorted task graph. */
        TaskPriorityGraph* taskPriorityGraph;
        /** A reference to the constraint model. */
        ConstraintModel* constraintModel;
    };
}

#endif
