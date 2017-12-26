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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {
    class TaskManager;
    /**
     * \brief Computes and applies the generalized forces that track the task
     * goals. 
     */
    class TaskBasedForce : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedForce, Force);
    public:
        /**
         * This object does not take ownership of the TaskManager. It must be
         * owned by the model (e.g. model.addComponent()).
         */
        TaskBasedForce(TaskManager* taskManager);
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
	/** A reference to the task manager. */
        TaskManager* taskManager;
	/** Stores the applied generalized forces. */
        mutable Storage appliedForces;
    }; 
} 

#endif
