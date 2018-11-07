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
#include <Settings.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(v[0], v[1], v[2]);
}

#define MAINTAIN_INITIAL_STATE(state, task)               \
    auto task ## x0 = fromVectorToVec3(task->x(state));   \
    auto task ## Goal = [&](const State& s) -> Vector {   \
        auto x = fromVectorToVec3(task->x(s));            \
        auto u = fromVectorToVec3(task->u(s));            \
        double kp = 1, kd = 10;                           \
        auto xd = task ## x0;                             \
        return Vector(kp * (xd - x) - kd * u);            \
    };                                                    \
    task->setGoal(task ## Goal);                          \

void predictiveSimulation() {
    throw runtime_error("Experimental: not functioning yet due to COMTask");
    const string example = "ExamplePredictiveSimulation";

    // load model
    // Model model("gait2392_simbody.osim");
    Model model(DATA_DIR + "/gait_model/lower_limb_floor_ideal_muscles.osim");
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
        if (model.getCoordinateSet()[i].getName().find("pelvis_") != std::string::npos ||
            model.getCoordinateSet()[i].getName().find("platform_") != std::string::npos) {
            S[i][i] = 1.0;
        }
    }
    cout << "Selection matrix: \n" << S << endl;
    auto taskDynamics = new TaskDynamics(constraintModel, S);
    model.addComponent(taskDynamics);

    // construct tasks
    /*auto comTask = new COMTask();
    taskDynamics->addTask(comTask, NULL);
    model.addComponent(comTask);*/

    auto pelvisTask = new OrientationTask("pelvis", Vec3(0));
    pelvisTask->setName("pelvis_task");
    taskDynamics->addTask(pelvisTask, NULL);
    model.addComponent(pelvisTask);

    auto footRTask = new OrientationTask("calcn_r", Vec3(0));
    footRTask->setName("foot_r_task");
    taskDynamics->addTask(footRTask, pelvisTask);
    model.addComponent(footRTask);

    auto footLTask = new OrientationTask("calcn_l", Vec3(0));
    footLTask->setName("foot_l_task");
    taskDynamics->addTask(footLTask, pelvisTask);
    model.addComponent(footLTask);

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
     /*auto comx0 = fromVectorToVec3(comTask->x(state));
     auto comGoal = [&](const State& s) -> Vector {
         auto x = fromVectorToVec3(comTask->x(s));
         auto u = fromVectorToVec3(comTask->u(s));
         double kp = 100, kd = 20;
         auto xd = comx0 + Vec3(0, -0.0 * sin(2 * Pi * (s.getTime())), 0);
         return Vector(kp * (xd - x) - kd * u);
     };
     comTask->setGoal(comGoal);*/

    MAINTAIN_INITIAL_STATE(state, pelvisTask);
    MAINTAIN_INITIAL_STATE(state, footRTask);
    MAINTAIN_INITIAL_STATE(state, footLTask);

    //simulate
    simulate(model, state, .7, true);

    // export results
    controller->printResults(example, DATA_DIR + "/results");
    bodyKinematics->printResults(example, DATA_DIR + "/results");
}

int main(int argc, char *argv[]) {
    try {
        predictiveSimulation();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}
