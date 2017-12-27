/**
 * \file The task based generalized force is a torque controller that uses task
 * space projection to evaluate the generalized forces that track the task goals
 * and to apply them to the model.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_BASED_FORCE_H
#define TASK_BASED_FORCE_H

#include <functional>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {
    /**
     * A function that evaluates the control strategy and returns the torques
     * that must be applied to actuate the model.
     *
     * @see TaskDynamics.h 
     */
    typedef std::function<SimTK::Vector(const SimTK::State& s)> ControlStrategy;
    /**
     * \brief Computes and applies the generalized forces that track the task
     * goals provided a function for evaluating the control strategy.
     *
     * @see TaskDynamics.h
     */
    class TaskBasedForce : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedForce, Force);
    public:
        /**
         * @param controlStrategy is a function that accepts the state and
         * returns a Vector of generalized forces.
         */
        TaskBasedForce(const ControlStrategy& controlStrategy);
	/**
	 * Prints the applied forces to a .sto file format.
	 *
	 * @param prefix is added to the file (useful for batch simulations).
	 * @param dir is the directory where the results will be stored.
	 */
        void printResults(std::string prefix, std::string dir);
    protected:
	/** Apply task forces as generalizedForces. */
        void computeForce(const SimTK::State& state,
                          SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                          SimTK::Vector& generalizedForces) const override;
	/** Perform some additional initialization. */
        void extendInitStateFromProperties(SimTK::State& s) const override;
    protected:
	/** Stores the applied generalized forces. */
        mutable Storage appliedForces;
	/** Control strategy. */
	ControlStrategy controlStrategy;
    }; 
} 

#endif
