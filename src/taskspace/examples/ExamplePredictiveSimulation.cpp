/**
 * @file ExamplePredictiveSimulation.cpp
 *
 * \brief
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <OpenSim/OpenSim.h>
#include <TaskSpace.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(v[0], v[1], v[2]);
}

void arm26Simulation() {
    cout << "Warning: The model geometry may not be visible if OpenSim's " <<
        "Geometry folder is missing. This does not affect the simulation" << endl;
    // load model
    // Model model("gait2392_simbody.osim");
    Model model("model_02-Scaled_to_subject.osim");
    model.setName("ExamplePredictiveSimulation");
#if USE_VISUALIZER == 1
    model.setUseVisualizer(true);
#endif

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // chose constraint model
    auto constraintModel = new UnconstraintModel();
    model.addComponent(constraintModel);

    // construct task dynamics
    auto taskDynamics = new TaskDynamics(constraintModel);
    model.addComponent(taskDynamics);

    // construct tasks
    auto torsoTask = new OrientationTask("torso", Vec3(0, 0, 0));
    taskDynamics->addTask(torsoTask, NULL);
    model.addComponent(torsoTask);

    /**
     * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t +
     * N_{g*}^T (f + b_c)\f$ as a callable function/closure ([&] captures the
     * current scope). This function accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        return data.tauTasks + 0 * data.NgT * (data.f + data.bc);
    };
    // define the controller (choose between a torque or muscle controller)
    auto controller = new TaskBasedTorqueController(controlStrategy);
    // auto controller = new TaskBasedComputedMuscleControl(controlStrategy);
    model.addController(controller);

    // build and initialize model
    for (int i = 0; i < model.getMuscles().getSize(); i++) {
        model.updMuscles().get(i).set_appliesForce(false);
    }
    auto& state = model.initSystem();

    /*model.updCoordinateSet().get("pelvis_tx").setValue(state, 0.05);
    model.updCoordinateSet().get("pelvis_ty").setValue(state, 1.05);
    model.updCoordinateSet().get("pelvis_tz").setValue(state, -0.1);*/

    // configure visualizer
#if USE_VISUALIZER == 1
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer()
        .setBackgroundType(Visualizer::BackgroundType::SolidColor);
    model.updMatterSubsystem().setShowDefaultGeometry(true);
#endif

    // define task goals as a function/closure
    /**
     * This implements a proportional-derivative (PD) tracking controller for
     * tracking the task goal. The task accepts a std::function which takes the
     * state and returns a Vector ([&] captures the current scope).
     */
    auto torsox0 = fromVectorToVec3(torsoTask->x(state));
    auto torsoGoal = [&](const State& s) -> Vector {
        auto x = fromVectorToVec3(torsoTask->x(s));
        auto u = fromVectorToVec3(torsoTask->u(s));
        double kp = 10, kd = 1;
        auto xd = torsox0;
        return Vector(kp * (xd - x) - kd * u);
    };
    torsoTask->setGoal(torsoGoal);

    //simulate
    simulate(model, state, .3, true);

    // export results
    controller->printResults("ExamplePredictiveSimulation", ".");
    bodyKinematics->printResults("ExamplePredictiveSimulation", ".");
}

int main(int argc, char *argv[]) {
    try {
        arm26Simulation();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}