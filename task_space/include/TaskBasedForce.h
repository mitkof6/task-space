#ifndef TASK_BASED_FORCE_H
#define TASK_BASED_FORCE_H

/**
 * \file This file contains the implementation of the task based generalized
 * force controller, that uses task space projection to evaluate the generalized
 * forces that track the task goals and to apply them to the model.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */


#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {
    class TaskManager;

    /**
     * Computes the generalized forces following a task oriented
     * controller approach.
     */
    class TaskBasedForce : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedForce, Force);
    public:

        /**
         * The controller does not takes ownership of the TaskDynamics object,
         * and the user should manually add this component to the model.
         */
        TaskBasedForce(TaskManager* taskManager);
        void printResults(std::string prefix, std::string dir);
    protected:
        // Force methods

        void computeForce(const SimTK::State& state,
                          SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                          SimTK::Vector& generalizedForces) const override;
        void extendInitStateFromProperties(SimTK::State& s) const override;
    protected:
        TaskManager* taskManager;
        mutable Storage forcesSto;
    }; 
} 

#endif
