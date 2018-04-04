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
    Model model("lower_limb_model_path.osim");
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

    // construct task dynamics and selection matrix for under-actuation
    Matrix S(model.getNumCoordinates(), model.getNumCoordinates());
    S = 1;
    for (int i = 0; i < model.getNumCoordinates(); i++) {
        if (model.getCoordinateSet()[i].getName().find("pelvis_") != std::string::npos) {
            S[i][i] = 0.0;
        }
    }
    cout << "Selection matrix: \n" << S << endl;
    auto taskDynamics = new TaskDynamics(constraintModel, S);
    model.addComponent(taskDynamics);

    // construct tasks
    // auto pelvisTask = new SpatialTask("pelvis", Vec3(0, 0, 0));
    auto pelvisTask = new COMTask();
    taskDynamics->addTask(pelvisTask, NULL);
    model.addComponent(pelvisTask);

    /**
     * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t +
     *  \f$ as a callable function/closure ([&] captures the current scope).
     *  This function accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        cout << data.tauTasks << endl;
        return data.tauTasks;
    };
    // define the controller (choose between a torque or muscle controller)
    auto controller = new TaskBasedTorqueController(controlStrategy);
    // auto controller = new TaskBasedComputedMuscleControl(controlStrategy);
    model.addController(controller);

    // build and initialize model
    auto& state = model.initSystem();

    // configure visualizer
#if USE_VISUALIZER == 1
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer()
        .setBackgroundType(Visualizer::BackgroundType::SolidColor);
    model.updMatterSubsystem().setShowDefaultGeometry(false);
#endif

    // define task goals as a function/closure
    /**
     * This implements a proportional-derivative (PD) tracking controller for
     * tracking the task goal. The task accepts a std::function which takes the
     * state and returns a Vector ([&] captures the current scope).
     */
    auto pelvisx0 = pelvisTask->x(state);
    auto pelvisGoal = [&](const State& s) -> Vector {
        auto x = pelvisTask->x(s);
        auto u = pelvisTask->u(s);
        double kp = 100, kd = 20;
        auto xd = pelvisx0;
        xd(3, 3) += Vector(3, &Vec3(0, -0.4 * sin(2 * Pi * (s.getTime() - 0.1)), 0)[0]);
        return kp * (xd - x) - kd * u;
    };
    pelvisTask->setGoal(pelvisGoal);

    //simulate
    simulate(model, state, 1.0, true);

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