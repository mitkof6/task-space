/**
 * \file A controller that computes the generalized forces to track the
 * task goals.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#ifndef TASK_BASED_TORQUE_CONTROLLER_H
#define TASK_BASED_TORQUE_CONTROLLER_H

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>

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
     * goals provided a function for evaluating the control strategy (@see
     * TaskDynamics.h).
     *
     * As this is a generalized force controller that evaluates the control
     * strategy and applies the forces to the model. As this is a Controller and
     * not a Force, CoordinateActuators are appended to the model and actuated
     * according to the control strategy.
     */
    class TaskBasedTorqueController : public Controller {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedTorqueController, Controller);
    public:
        /**
         * @param controlStrategy is a function that accepts the state and
         * returns a Vector of generalized forces.
         */
        TaskBasedTorqueController(const ControlStrategy& controlStrategy);
        /**
         * Prints the applied forces to a .sto file format.
         *
         * @param prefix is added to the file (useful for batch simulations).
         * @param dir is the directory where the results will be stored.
         */
        void printResults(std::string prefix, std::string dir);
    protected:
        /** Controller. */
        void computeControls(const SimTK::State& s,
                             SimTK::Vector& controls) const override;
        /** Add CoordinateActuatros for controlling the model. */
        void extendConnectToModel(Model& model) override;
    private:
        /** Stores the applied generalized forces. */
        mutable Storage appliedForces;
        /** Control strategy. */
        ControlStrategy controlStrategy;
    };
}

#endif
