/**
 * \file An example of utilizing the task based projection in order to control a
 * model in task space. In this example a block is created and a position task
 * is assigned. The goal is prescribed using a PD tracking controller and a
 * forward simulation is performed.
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
    return Vec3(v[0], v[1], v[2]);
}

void taskBasedControl() {
    // create model
    Model model;
    model.setUseVisualizer(true);
    Vec3 halfLength(0.2, 0.2, 0.2);
    auto block = new OpenSim::Body("block", 1, Vec3(0),
                                   Inertia::brick(halfLength));
    auto blockGeom = new OpenSim::Brick(halfLength);
    block->attachGeometry(blockGeom);
    model.addBody(block);
    auto joint = new FreeJoint("joint",
                               model.getGround(), Vec3(0), Vec3(0),
                               *block, Vec3(0), Vec3(0));
    model.addJoint(joint);

    // add coordinate console reporter
    auto reporter = new ConsoleReporter();
    reporter->set_report_time_interval(0.1);
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationX)
                          .getOutput("value"), "X");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationY)
                          .getOutput("value"), "Y");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationZ)
                          .getOutput("value"), "Z");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation1X)
                          .getOutput("value"), "thetaX");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation2Y)
                          .getOutput("value"), "thetaY");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation3Z)
                          .getOutput("value"), "thetaZ");
    model.addComponent(reporter);

    // construct task priority graph
    TaskPriorityGraph graph;
    auto task = new PositionTask("block", Vec3(0));
    graph.addTask(task, NULL);
    model.addComponent(task);

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
    auto state = model.initSystem();

    // configure visualizer
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer()
        .setBackgroundType(Visualizer::BackgroundType::SolidColor);
    model.updMatterSubsystem().setShowDefaultGeometry(true);

    // initial configuration
    joint->updCoordinate(FreeJoint::Coord::TranslationY).setValue(state, 0.5);

    // define task goal function/closure
    auto x0 = fromVectorToVec3(task->x(state));
    /**
     * This implements a proportional-derivative (PD) tracking controller for
     * tracking the task goal. The task accepts a std::function which takes the
     * state and returns a Vector.
     */
    auto pd = [&](const State& s) -> Vector { // [&] captures the current scope
        auto x = fromVectorToVec3(task->x(s));
        auto u = fromVectorToVec3(task->u(s));
        double kp = 100, kd = 20;
        auto xd = x0 + Vec3(sin(2 * Pi * s.getTime()), 0, 0);
        auto ud = Vec3(2 * Pi * cos(2 * Pi * s.getTime()), 0, 0);
        auto ad = Vec3(-pow(2 * Pi, 2) * sin(2 * Pi * s.getTime()), 0, 0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    task->setGoal(pd);

    //simulate
    simulate(model, state, 2);

    // export results
    controller->printResults("ExampleTaskBasedControl", ".");
    reporter->print("ExampleTaskBasedContro_Reporter.sto");
}

int main(int argc, char *argv[]) {
    try {
        taskBasedControl();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}