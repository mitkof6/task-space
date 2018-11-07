/**
 * @file ExampleUpperLimb.cpp
 *
 * \brief Control of the MoBL 2016 upper limb model.
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

void predictiveSimulation() {
    const string example = "ExampleUpperLimb";

    // load model
    Model model(DATA_DIR + "/mobl/mobl_2016_ideal_muscles.osim");
#if USE_VISUALIZER == 1
    model.setUseVisualizer(true);
#endif

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // chose constraint model
    auto constraintModel = new AghiliModel();
    model.addComponent(constraintModel);

    // construct task dynamics and selection matrix for under-actuation
    Matrix S(model.getNumCoordinates(), model.getNumCoordinates());
    S = 1;
    cout << "Selection matrix: \n" << S << endl;
    auto taskDynamics = new TaskDynamics(constraintModel, S);
    model.addComponent(taskDynamics);

    auto marker = model.getMarkerSet().get("end_effector");
    auto handTask = new SpatialTask(marker.getParentFrameName().substr(3),
                                    marker.get_location());
    handTask->setName("hand_task");
    taskDynamics->addTask(handTask, NULL);
    model.addComponent(handTask);

    /**
     * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t +
     *  \f$ as a callable function/closure ([&] captures the current scope).
     *  This function accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        return data.tauTasks + data.NgT * (data.f + data.bc);
    };
    // define the controller (choose between a torque or muscle controller)
    auto controller = new TaskBasedTorqueController(controlStrategy);
    model.addController(controller);

    // build and initialize model
    auto& state = model.initSystem();

    model.updCoordinateSet().get("elv_angle")
        .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet().get("shoulder_elv")
        .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet().get("shoulder_rot")
        .setValue(state, convertDegreesToRadians(-20));
    model.updCoordinateSet().get("elbow_flexion")
        .setValue(state, convertDegreesToRadians(95));
    model.updCoordinateSet().get("pro_sup")
        .setValue(state, convertDegreesToRadians(75));
    model.updCoordinateSet().get("flexion")
        .setValue(state, convertDegreesToRadians(0));

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
    auto handx0 = handTask->x(state);
    auto handGoal = [&](const State& s) -> Vector {
        const double kp = 100, kd = 20;
        const double A = 0.1;
        const double t = s.getTime();
        auto x = handTask->x(s);
        auto u = handTask->u(s);
        auto xpd = Vec3(0,
                        A * sin(Pi * t)*sin(2 * Pi * t),
                        A * sin(2 * Pi * t) * cos(Pi * t));
        auto upd = Vec3(0,
                        A * Pi * (-sin(Pi * t) + 3 * sin(3 * Pi * t)) / 2,
                        A * Pi * (cos(Pi * t) + 3 * cos(3 * Pi * t)) / 2);
        auto apd = Vec3(0,
                        A * Pi *(-sin(Pi * t) + 3 * sin(3 * Pi * t)) / 2,
                        -A * Pi * Pi * (sin(Pi * t) + 9 * sin(3 * Pi * t)) / 2);
        auto xd = handx0;
        xd(3, 3) += Vector(xpd);
        auto ud = Vector(6, 0.0);
        ud(3, 3) += Vector(upd);
        auto ad = Vector(6, 0.0);
        ad(3, 3) += Vector(apd);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    handTask->setGoal(handGoal);

    //simulate
    simulate(model, state, 2.0, true);

    // export results
    controller->printResults(example, DATA_DIR + "/results");
    bodyKinematics->printResults(example, DATA_DIR + "/results");
}

int main(int argc, char *argv[]) {
    try {
        predictiveSimulation();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}
