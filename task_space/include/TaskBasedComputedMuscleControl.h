/**
* \file A controller that computes the actuators (muscles) excitations through
*  an optimization procedure in order to track the task goals. For more details
*  please refer to Section II(I).
*
* @author Dimitar Stanev <jimstanev@gmail.com>
*
* @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
* href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
*/
#ifndef TASK_BASED_COMPUTED_MUSCLE_CONTROL_H
#define TASK_BASED_COMPUTED_MUSCLE_CONTROL_H

#include <functional>
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
    typedef
        std::function<SimTK::Vector(const SimTK::State& s)> ControlStrategy;
    /**
     * \brief Computes the excitations using an optimization procedure that
     * track the task goals provided a function for evaluating the control
     * strategy (@see TaskDynamics.h).
     */
    class TaskBasedComputedMuscleControl : public Controller {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedComputedMuscleControl,
                                        Controller);
    public:
        /**
        * @param controlStrategy is a function that accepts a state and
        * returns a Vector of generalized forces.
        */
        TaskBasedComputedMuscleControl(const ControlStrategy& controlStrategy);
        /**
        * Prints the controls (reserve and model actuators) to a .sto file
        * format.
        *
        * @param prefix is added to the file (useful for batch simulations).
        * @param dir is the directory where the results will be stored.
        */
        void printResults(std::string prefix, std::string dir);
    protected:
        /**
         * Computes the actuator controls. If the optimization is infeasible
         * reserve actuators are used to compensate for the residual forces.
         *
         * \$f \tau_{r} = \tau - R(q) f_m \odot \alpha \f$
         */
        void computeControls(const SimTK::State& s,
                             SimTK::Vector& controls) const override;
        /** Adds reserve actuators and initializes the optimization. */
        void extendConnectToModel(Model& model) override;
    private:
        /** Control strategy. */
        ControlStrategy controlStrategy;
        /** Stores the computed controls. */
        mutable DataTable controlsDataTable;
        /** Optimizer (method interior point). */
        SimTK::ReferencePtr<SimTK::Optimizer> optimizer;
        /** The optimization system. */
        SimTK::ReferencePtr<MuscleOptimizationTarget> target;
        /**
        * Gets an initial activation for the optimization from the
        * controlsDatatable. This function ensures that the DataTable contains
        * time increasing entries.
        */
        SimTK::Vector getInitialActivations(const SimTK::State& s) const;
    };
    /**
    * \brief A SimTK::OptimizerSystem for formulating minimum muscle effort
    * optimization objectives.
    *
    * The optimization problem is defined as follows
    *
    * \f$ \underset{\alpha}{\text{minimize}} \sum_{i=1}^{m} |\alpha_i|^e \f$
    *
    * \f$  \text{subject to} \tau = R(q) f_m \odot \alpha) \f$
    */
    class MuscleOptimizationTarget : public SimTK::OptimizerSystem {
    public:
        /** Optimization parameters. */
        struct OptimizationParameters {
            int numActuators;
            int numConstraints;
            int activationExponent;
        };
        MuscleOptimizationTarget(OptimizationParameters parameters);
        /** This function must be called before performing an optimization. */
        void prepareToOptimize(const SimTK::Vector& desiredTau,
                               const SimTK::Matrix& momentArm,
                               const SimTK::Vector& maxForce);
        /** Report solution statistics. */
        void evaluateObjective(const SimTK::State& s, const SimTK::Vector& x);
        /** \f$ \sum_{i=1}^{m} |\alpha_i|^e \f$ */
        int objectiveFunc(const SimTK::Vector& x, bool newPar,
                          SimTK::Real& f) const override;
        /** \f$ \sum_{i=1}^{m} e |\alpha_i|^{e-1} \f$ */
        int gradientFunc(const SimTK::Vector& x, bool newPar,
                         SimTK::Vector& gradient) const override;
        /** \f$ \tau - R(q) f_m \odot \alpha = 0 \f$ */
        int constraintFunc(const SimTK::Vector& x, bool newPar,
                           SimTK::Vector& constraints) const override;
        /** \f$ J[i][j] = - R(q)[i, j] f_m[j] \f$ */
        int constraintJacobian(const SimTK::Vector& x, bool newPar,
                               SimTK::Matrix& jac) const override;
    private:
        OptimizationParameters parameters;
        /** Muscle moment arm matrix. */
        SimTK::Matrix R;
        /** Maximum muscle forces and desired generalized forces. */
        SimTK::Vector Fmax, tau;
    };
}

#endif