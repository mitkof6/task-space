/**
 * @file KinematicTask.h
 *
 * \brief Implementation of the kinematic task primitives (i.e. position,
 * orientation and spatial which is both). For more details please refer to
 * Section II(D).
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef KINEMATIC_TASK_H
#define KINEMATIC_TASK_H

#include <string>
#include <functional>
#include "internal/TaskSpaceExports.h"
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /**
     * A task goal (typically acceleration) is prescribed by providing a
     * function or a closure through setGoal. The provided function must be of
     * type std::function, accepting the State and returning a Vector of task
     * accelerations.
     *
     * Example:
     * \code{.cpp}
     * TaskPriorityGraph graph;
     * auto task = new PositionTask("block", Vec3(0));
     * graph.addTask(task, NULL);
     * model.addComponent(task);
     * ...
     * auto x0 = fromVectorToVec3(task->x(state));
     * auto pd = [&](const State& s) -> Vector {
     *     auto x = fromVectorToVec3(task->x(s));
     *     auto u = fromVectorToVec3(task->u(s));
     *     double kp = 100, kd = 20;
     *     auto xd = x0 + Vec3(sin(2 * Pi * s.getTime()), 0, 0);
     *     auto ud = Vec3(2 * Pi * cos(2 * Pi * s.getTime()), 0, 0);
     *     auto ad = Vec3(-pow(2 * Pi, 2) * sin(2 * Pi * s.getTime()), 0, 0);
     *     return Vector(ad + kp * (xd - x) + kd * (ud - u));
     * };
     * task->setGoal(pd);
     * \endcode
     *
     * @see ExampleTaskBasedControl.cpp
     */
    typedef std::function<SimTK::Vector(const SimTK::State&)> TaskGoal;
    /**
     * \brief An abstract class for the kinematic task primitives.
     *
     * The task space position (\f$ x_t\f$) is a function of the generalized
     * coordinates (\f$ q \f$)
     *
     * \f$ x_t = g(q),\ q \in \Re^n \f$                                     (1)
     *
     * The first and second derivatives of Eq. (1) (the dot notation depicts a
     * derivative with respect to time) is given by
     *
     * \f$ \dot{x}_t = J(q) \dot{q} \f$
     *
     * \f$ \ddot{x}_t = \dot{J}(q) \dot{q} + J(q) \ddot{q} \f$
     *
     * where \f$ J = J(q) \f$ is the task Jacobian matrix and the term \f$
     * \dot{J} \dot{q} = - b_t \f$ accounts for the time varying effect of the
     * Jacobian matrix.
     *
     * @see TaskProjection.h
     */
    class TaskSpace_API KinematicTask : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(KinematicTask, ModelComponent);
    public:
        /**
         * Abstract kinematic task.
         *
         * @param body the body name
         * @param offset offset vector in body frame
         */
        KinematicTask(std::string body, SimTK::Vec3 offset);
        /**
         * Set the task goal by providing a callable std::function which
         * accepts the State and returns a Vector.
         *
         * @param goal a function or closure
         */
        void setGoal(const TaskGoal& goal);
        /** Evaluates the TaskGoal */
        SimTK::Vector getGoal(const SimTK::State& s) const;
        /** Calculates the Jacobian matrix (\f$ J_t \f$) */
        virtual SimTK::Matrix J(const SimTK::State& s) const = 0;
        /** Calculates the task bias term \f$ b_t = - \dot{J} \dot{q} \f$ */
        virtual SimTK::Vector b(const SimTK::State& s) const = 0;
        /**
         * Calculates the task position.
         *
         *  \throws Exception if the state is not realized to Stage::Position.
         */
        virtual SimTK::Vector x(const SimTK::State& s) const = 0;
        /**
         * Calculates the task velocity.
         *
         *  \throws Exception if the state is not realized to Stage::Velocity.
         */
        virtual SimTK::Vector u(const SimTK::State& s) const = 0;
        /**
         * Calculates the task acceleration.
         *
         *  \throws Exception if the state is not realized to
         *  Stage::Acceleration.
         */
        virtual SimTK::Vector a(const SimTK::State& s) const = 0;
        /** cout << task << endl;*/
        friend std::ostream& operator<<(std::ostream& os,
                                        const KinematicTask& k) {
            return os << "[Body: " << k.body << ", offset: " << k.offset << "]"
                << std::endl;
        };
    protected:
        /** Name of the body that the task is attached to. */
        std::string body;
        /** Offset of the task from the body's local frame. */
        SimTK::Vec3 offset;
        /** A callable function or closure that evaluates the task goal. */
        TaskGoal goal;
    };
    /**
     * \brief Position task primitive.
     *
     * For position tasks the following hold true:
     *
     * \f$ x_t = x \in \Re^3 \f$
     *
     * \f$ J_t \in \Re^{3 \times n} \f$
     *
     * \f$ b_t \in \Re^3 \f$
     *
     * @see KinematicTask
     */
    class TaskSpace_API PositionTask : public KinematicTask {
        OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, KinematicTask);
    public:
        /**
         * Position task.
         *
         * @param body the body name
         * @param offset offset vector in body frame
         */
        PositionTask(std::string body, SimTK::Vec3 offset);
        SimTK::Matrix J(const SimTK::State& s) const override;
        SimTK::Vector b(const SimTK::State& s) const override;
        SimTK::Vector x(const SimTK::State& s) const override;
        SimTK::Vector u(const SimTK::State& s) const override;
        SimTK::Vector a(const SimTK::State& s) const override;
    };
    /**
     * \brief Orientation task primitive.
     *
     * For orientation tasks the following hold true:
     *
     * \f$ x_t = \theta \in \Re^3 \f$
     *
     * \f$ J_t \in \Re^{3 \times n} \f$
     *
     * \f$ b_t \in \Re^3 \f$
     *
     * @see KinematicTask
     */
    class TaskSpace_API OrientationTask : public KinematicTask {
        OpenSim_DECLARE_CONCRETE_OBJECT(OrientationTask, KinematicTask);
    public:
        /**
         * Orientation task.
         *
         * @param body the body name
         * @param offset offset vector in body frame
         */
        OrientationTask(std::string body, SimTK::Vec3 offset);
        SimTK::Matrix J(const SimTK::State& s) const override;
        SimTK::Vector b(const SimTK::State& s) const override;
        SimTK::Vector x(const SimTK::State& s) const override;
        SimTK::Vector u(const SimTK::State& s) const override;
        SimTK::Vector a(const SimTK::State& s) const override;
    };
    /**
     * \brief Spatial task primitive (both position and orientation).
     *
     * For spatial tasks the following hold true:
     *
     * \f$ x_t = [\theta, x] \in \Re^6 \f$
     *
     * \f$ J_t \in \Re^{6 \times n} \f$
     *
     * \f$ b_t \in \Re^6 \f$
     *
     * @see KinematicTask
     */
    class TaskSpace_API SpatialTask : public KinematicTask {
        OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTask, KinematicTask);
    public:
        /**
         * Spatial task.
         *
         * @param body the body name
         * @param offset offset vector in body frame
         */
        SpatialTask(std::string body, SimTK::Vec3 offset);
        SimTK::Matrix J(const SimTK::State& s) const override;
        SimTK::Vector b(const SimTK::State& s) const override;
        SimTK::Vector x(const SimTK::State& s) const override;
        SimTK::Vector u(const SimTK::State& s) const override;
        SimTK::Vector a(const SimTK::State& s) const override;
    };
}

#endif
