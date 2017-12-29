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

void reaching() {
    // create model
    Model model("arm26.osim");
    model.setUseVisualizer(true);
    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);
    //model.addComponent(reporter);
    // construct task priority graph
    TaskPriorityGraph graph;
    auto handTask = new PositionTask("r_ulna_radius_hand", Vec3(0.02, -0.4, 0.1));
    graph.addTask(handTask, NULL);
    model.addComponent(handTask);
    auto elbowTask = new PositionTask("r_ulna_radius_hand", Vec3(0.0));
    graph.addTask(elbowTask, handTask);
    model.addComponent(elbowTask);
    // chose constraint model
    auto constraintModel = new UnconstraintModel();
    model.addComponent(constraintModel);
    // construct task dynamics
    auto taskDynamics = new TaskDynamics(&graph, constraintModel);
    model.addComponent(taskDynamics);
    /**
     * Define the control strategy \f$ \tau = \sum_{t=1}^g J_{t|t-1*}^T f_t \f$
     * as a callable function/closure ([&] captures the current scope). This
     * function accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        return data.tauTasks;
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
    // define task goal function/closure
    auto handx0 = fromVectorToVec3(handTask->x(state));
    /**
     * This implements a proportional-derivative (PD) tracking controller for
     * tracking the task goal. The task accepts a std::function which takes the
     * state and returns a Vector.
     */
    auto handGoal = [&](const State& s) -> Vector { // [&] captures the current scope
        auto x = fromVectorToVec3(handTask->x(s));
        auto u = fromVectorToVec3(handTask->u(s));
        double kp = 100, kd = 20;
        auto xd = handx0 + Vec3(0.2 * sin(2 * Pi * s.getTime()), 0, 0);
        auto ud = Vec3(2 * Pi * 0.2 * cos(2 * Pi * s.getTime()), 0, 0);
        auto ad = Vec3(-pow(2 * Pi, 2) * 0.2 * sin(2 * Pi * s.getTime()), 0, 0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    handTask->setGoal(handGoal);
    auto elbowx0 = fromVectorToVec3(elbowTask->x(state));
    auto elbowGoal = [&](const State& s) -> Vector { // [&] captures the current scope
        auto x = fromVectorToVec3(elbowTask->x(s));
        auto u = fromVectorToVec3(elbowTask->u(s));
        double kp = 10, kd = 2;
        auto xd = elbowx0;
        auto ud = Vec3(0);
        auto ad = Vec3(0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    elbowTask->setGoal(elbowGoal);
    //simulate
    simulate(model, state, 2);
    // export results
    controller->printResults("ExampleReaching", ".");
    bodyKinematics->printResults("ExampleReaching", ".");
}

int main(int argc, char *argv[]) {
    try {
        reaching();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();e
        return -1;
    }
    return 0;
}
