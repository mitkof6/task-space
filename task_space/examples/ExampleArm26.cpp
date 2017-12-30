/**
 * \file TODO
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <OpenSim/OpenSim.h>
#include "TaskSpace.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(&v(0, 3)[0]);
}

void arm26Simulation() {
    // load model
    Model model("arm26.osim");
    model.setUseVisualizer(true);

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // construct task priority graph 
    TaskPriorityGraph graph;
    auto humerusTask = new OrientationTask("r_humerus", Vec3(0, -0.18, 0));
    graph.addTask(humerusTask, NULL); // humerus has the highest priority
    model.addComponent(humerusTask);
    auto ulnaTask = new OrientationTask("r_ulna_radius_hand", Vec3(0.02, -0.4, 0.1));
    graph.addTask(ulnaTask, humerusTask); // ulna is prioritized by humerus
    model.addComponent(ulnaTask);

    // chose constraint model
    auto constraintModel = new UnconstraintModel();
    model.addComponent(constraintModel);

    // construct task dynamics
    auto taskDynamics = new TaskDynamics(&graph, constraintModel);
    model.addComponent(taskDynamics);

    /**
    * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t +
    * N_{g*}^T (f + b_c)\f$ as a callable function/closure ([&] captures the
    * current scope). This function accepts the state and returns a Vector.
    */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        return data.tauTasks + data.NgT * (data.f + data.bc);
    };
    // construct a torque controller and supply the control strategy
    auto controller = new TaskBasedTorqueController(controlStrategy);
    model.addController(controller);

    // build and initialize model
    auto& state = model.initSystem();

    // configure visualizer
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer()
        .setBackgroundType(Visualizer::BackgroundType::SolidColor);
    model.updMatterSubsystem().setShowDefaultGeometry(true);

    // define task goals as a function/closure
    // make elbow humerus motionless
    auto humerusx0 = fromVectorToVec3(humerusTask->x(state));
    /**
    * This implements a proportional-derivative (PD) tracking controller for
    * tracking the task goal. The task accepts a std::function which takes the
    * state and returns a Vector ([&] captures the current scope).
    */
    auto humerusGoal = [&](const State& s) -> Vector { 
        auto x = fromVectorToVec3(humerusTask->x(s));
        auto u = fromVectorToVec3(humerusTask->u(s));
        double kp = 100, kd = 20;
        auto xd = humerusx0;
        auto ud = Vec3(0);
        auto ad = Vec3(0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    humerusTask->setGoal(humerusGoal);

    // make the ulna rotate around z-axis
    auto ulnax0 = fromVectorToVec3(ulnaTask->x(state));
    auto ulnaGoal = [&](const State& s) -> Vector {
        auto x = fromVectorToVec3(ulnaTask->x(s));
        auto u = fromVectorToVec3(ulnaTask->u(s));
        double kp = 100, kd = 20;
        auto xd = ulnax0 + Vec3(0, 0, Pi / 4 * (sin(1 * Pi * s.getTime()) + 1));
        auto ud = Vec3(0, 0, 1 * Pi * Pi / 4 * cos(1 * Pi * s.getTime()));
        auto ad = Vec3(0, 0, -pow(1 * Pi, 2) * Pi / 4 * sin(1 * Pi * s.getTime()));
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    ulnaTask->setGoal(ulnaGoal);

    //simulate
    simulate(model, state, 2);

    // export results
    controller->printResults("ExampleArm26", ".");
    bodyKinematics->printResults("ExampleArm26", ".");
}

int main(int argc, char *argv[]) {
    try {
        arm26Simulation();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}