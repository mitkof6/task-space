/**
 * @file ExampleBicycle.cpp
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
#include <Settings.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(v[0], v[1], v[2]);
}

SimTK::State simulate2(Model& model,
                       const SimTK::State& initialState,
                       double finalTime,
                       bool saveStatesFile = false) {
    // Returned state begins as a copy of the initial state
    SimTK::State state = initialState;
    SimTK::Visualizer::InputSilo* silo;

    bool simulateOnce = true;

    // Configure the visualizer.
    if (model.getUseVisualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        // We use the input silo to get key presses.
        silo = &model.updVisualizer().updInputSilo();

        SimTK::DecorativeText help("Press any key to start a new simulation; "
                                   "ESC to quit.");
        help.setIsScreenText(true);
        viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);

        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        std::cout << "A visualizer window has opened." << std::endl;

        // if visualizing enable replay
        simulateOnce = false;
    }

    // Simulate until the user presses ESC (or enters 'q' if visualization has
    // been disabled).
    do {
        if (model.getUseVisualizer()) {
            // Get a key press.
            silo->clear(); // Ignore any previous key presses.
            unsigned key, modifiers;
            silo->waitForKeyHit(key, modifiers);
            if (key == SimTK::Visualizer::InputListener::KeyEsc) { break; }
        }

        // reset the state to the initial state
        state = initialState;
        // Set up manager and simulate.
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        integrator.setConstraintTolerance(0.5);
        Manager manager(model, integrator);
        state.setTime(0.0);
        manager.initialize(state);
        manager.integrate(finalTime);

        // Save the states to a storage file (if requested).
        if (saveStatesFile) {
            manager.getStateStorage().print(model.getName() + "_states.sto");
        }
    } while (!simulateOnce);

    return state;
}

void cyclingSimulation() {
    const string example = "ExampleCycling";

    cout << "Warning: The model geometry may not be visible if OpenSim's " <<
        "Geometry folder is missing. This does not affect the simulation" << endl;
    // load model
    // Model model("gait2392_simbody.osim");
    Model model(DATA_DIR + "/bicycle/bicycle_v08.osim");
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
        S[i][i] = 1.0;
        /*if (model.getCoordinateSet()[i].getName().find("gear_rotation") != std::string::npos ||
            model.getCoordinateSet()[i].getName().find("pedal_rotation_") != std::string::npos) {
            S[i][i] = 0;
        }*/
    }
    cout << "Selection matrix: \n" << S << endl;
    auto taskDynamics = new TaskDynamics(constraintModel, S);
    model.addComponent(taskDynamics);

    // construct tasks
    /*auto comTask = new COMTask();
    taskDynamics->addTask(comTask, NULL);
    model.addComponent(comTask);*/

    auto gearsTask = new OrientationTask("gears", Vec3(0));
    gearsTask->setName("gears_task");
    taskDynamics->addTask(gearsTask, NULL);
    model.addComponent(gearsTask);

    /**
     * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t +
     *  \f$ as a callable function/closure ([&] captures the current scope).
     *  This function accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        // cout << data.tauTasks << endl;
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
    auto gearsx0 = fromVectorToVec3(gearsTask->x(state));
    auto gearsGoal = [&](const State& s) -> Vector {
        auto x = fromVectorToVec3(gearsTask->x(s));
        auto u = fromVectorToVec3(gearsTask->u(s));
        double kp = 100, kd = 20;
        double mag = 10;
        double f = 2 * 4;
        auto xd = gearsx0 + Vec3(mag * sin(f * Pi * s.getTime()), 0, 0);
        auto ud = Vec3(f * Pi * mag * cos(f * Pi * s.getTime()), 0, 0);
        auto ad = Vec3(-pow(f * Pi, 2) * mag * sin(f * Pi * s.getTime()), 0, 0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    gearsTask->setGoal(gearsGoal);

    //simulate
    // simulate(model, state, .7, true);
    simulate2(model, state, 1.0, true);

    // export results
    controller->printResults(example, DATA_DIR + "/results");
    bodyKinematics->printResults(example, DATA_DIR + "/results");
}

int main(int argc, char *argv[]) {
    try {
        cyclingSimulation();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}