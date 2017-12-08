#ifndef KINEMATIC_TASK_H
#define KINEMATIC_TASK_H

#include <string>
#include <OpenSim/Simulation/Model/ModelComponent.h>

/**
 * \file This file contains the implementation of kinematic tasks.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

namespace OpenSim {

    /**
     * \brief An abstract class for kinematic tasks (e.g. position, orientation
     * and spatial).
     *
     * The task space position (\f$ x_t\f$) is a function of the generalized
     * coordinates (\f$ q \f$)
     *
     * \f$ x_t = g(q),\ q \in \Re^n \f$ (1)
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
     */
    class KinematicTask : public ModelComponent {
	OpenSim_DECLARE_ABSTRACT_OBJECT(KinematicTask, ModelComponent);
    public:
	KinematicTask(std::string body, SimTK::Vec3 offset);
	void setGoal(const SimTK::Vector& goal);
	const SimTK::Vector& getGoal() const;
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
    protected:
	/** Name of the body that the task is attached to. */
	std::string body;
	/** Offset of the task from the body's local frame. */
	SimTK::Vec3 offset;
	/** The goal of this task (task acceleration). */
	SimTK::Vector goal;
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
    class PositionTask : public KinematicTask {
	OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, KinematicTask);
    public:
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
    class OrientationTask : public KinematicTask {
	OpenSim_DECLARE_CONCRETE_OBJECT(OrientationTask, KinematicTask);
    public:
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
    class SpatialTask : public KinematicTask {
	OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTask, KinematicTask);
    public:
	SpatialTask(std::string body, SimTK::Vec3 offset);
	SimTK::Matrix J(const SimTK::State& s) const override;
	SimTK::Vector b(const SimTK::State& s) const override;
	SimTK::Vector x(const SimTK::State& s) const override;
	SimTK::Vector u(const SimTK::State& s) const override;
	SimTK::Vector a(const SimTK::State& s) const override;
    };
}

#endif
