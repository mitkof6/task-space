/**
* \file TODO
*
* @author Dimitar Stanev <jimstanev@gmail.com>
*
* @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
* href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
*/
#ifndef TASK_BASED_COMPUTED_MUSCLE_CONTROL_H
#define TASK_BASED_COMPUTED_MUSCLE_CONTROL_H

#include <functional>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {
    class MuscleOptimizationTarget;
    /**
    * A function that evaluates the control strategy and returns the torques
    * that must be applied to actuate the model.
    *
    * @see TaskDynamics.h
    */
    typedef std::function<SimTK::Vector(const SimTK::State& s)> ControlStrategy;
    /**
     *
     */
    class TaskBasedComputedMuscleControl : public Controller {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedComputedMuscleControl, Controller);
    public:
        /**
        * @param controlStrategy is a function that accepts the state and
        * returns a Vector of generalized forces.
        */
        TaskBasedComputedMuscleControl(const ControlStrategy& controlStrategy);
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
        /** Adds reserve actuators. */
        void extendConnectToModel(Model& model) override;
    private:
        /** Stores the computed controls. */
        mutable Storage controlStorage;
        /** Control strategy. */
        ControlStrategy controlStrategy;
        /** Optimizer */
        SimTK::ReferencePtr<SimTK::Optimizer> optimizer;
        SimTK::ReferencePtr<MuscleOptimizationTarget> target;
    };

    class MuscleOptimizationTarget : public SimTK::OptimizerSystem {
    public:
        struct OptimizationParameters {
            int numActuators;
            int numResidualActuators;
            int gamma = 1000;
            int activationExponent = 2;
            int maxResidualForce = 1000;
        };

        MuscleOptimizationTarget(OptimizationParameters parameters);

        void prepareToOptimize(const SimTK::Vector& desiredTau,
                               const SimTK::Matrix& momentArm,
                               const SimTK::Vector& maxForce);

        void evaluateObjective(const SimTK::State& s, const SimTK::Vector& x);
    protected:
        int objectiveFunc(const SimTK::Vector& x, bool newPar,
                          SimTK::Real& f) const override;
        int gradientFunc(const SimTK::Vector& x, bool newPar,
                         SimTK::Vector& gradient) const override;
        int constraintFunc(const SimTK::Vector& x, bool newPar,
                           SimTK::Vector& constraints) const override;
        int constraintJacobian(const SimTK::Vector& x, bool newPar,
                               SimTK::Matrix& jac) const override;
    private:
        OptimizationParameters parameters;
        SimTK::Matrix R;
        SimTK::Vector Fmax, tau;
    };
}

#endif