/**
 * \file An example of task space control for a model that is build using
 * absolute coordinates and constraints. For more details please refer to
 * Section Supplementary Material(D).
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <iostream>
#include <OpenSim/OpenSim.h>
#include <TaskSpace.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(&v(0, 3)[0]);
}

void absoluteCoordinates() {
    // parameters
    const double q1 = -45.0;
    const double q2 = 90;
    const double l = 1;
    const string taskBodyName = "body2";
    // model
    Model model;
    // configure the visualizer
    model.setUseVisualizer(true);
    // construct model
    auto ground = &model.updGround();
    // body1
    double body1_m = 1, body1_length = 1, body1_radius = 0.03;
    Vec3 body1_com = Vec3(0);
    Inertia body1_I = body1_m * Inertia::cylinderAlongY(
        body1_radius, body1_length);
    auto body1_body = new OpenSim::Body("body1", body1_m, body1_com, body1_I);
    auto body1_geom = new OpenSim::Cylinder(body1_radius, body1_length / 2);
    body1_geom->setName("body1_cylinder");
    body1_body->attachGeometry(body1_geom);
    Vec3 body1_distal(0, -body1_length / 2, 0);
    Vec3 body1_proximal(0, body1_length / 2, 0);
    auto ground_body1 = new OpenSim::FreeJoint(
        "ground_body1", *ground,
        Vec3(0), Vec3(0), *body1_body, body1_distal, Vec3(0));
    model.addBody(body1_body);
    model.addJoint(ground_body1);
    // body2
    double body2_m = 1, body2_length = 1, body2_radius = 0.03;
    Vec3 body2_com = Vec3(0);
    Inertia body2_I = body2_m * Inertia::cylinderAlongY(
        body2_radius, body2_length);
    auto body2_body = new OpenSim::Body("body2", body2_m, body2_com, body2_I);
    auto body2_geom = new OpenSim::Cylinder(body2_radius, body2_length / 2);
    body2_geom->setName("body2_cylinder");
    body2_body->attachGeometry(body2_geom);
    Vec3 body2_distal(0, -body2_length / 2, 0);
    Vec3 body2_proximal(0, body2_length / 2, 0);
    auto body1_body2 = new OpenSim::FreeJoint(
        "body1_body2", *body1_body,
        body1_proximal, Vec3(0), *body2_body, body2_distal, Vec3(0));
    model.addBody(body2_body);
    model.addJoint(body1_body2);
    // connect the two free bodies
    auto pointConstraint1 = new PointConstraint(*ground, Vec3(0),
                                                *body1_body, body1_distal);
    model.addConstraint(pointConstraint1);
    auto pointConstraint2 = new PointConstraint(*body1_body, body2_proximal,
                                                *body2_body, body2_distal);
    model.addConstraint(pointConstraint2);
    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);
    // construct task priority graph
    TaskPriorityGraph graph;
    auto task = new PositionTask(taskBodyName, Vec3(0, l, 0)); // end effector
    graph.addTask(task, NULL);
    model.addComponent(task);
    // chose constraint model
    auto constraintModel = new AghiliModel();
    model.addComponent(constraintModel);
    // construct task manager
    auto manager = new TaskManager(&graph, constraintModel);
    model.addComponent(manager);
    // chose controller
    auto forceController = new TaskBasedForce(manager);
    model.addForce(forceController);
    // *************************************************************************
    // build and initialize model
    auto state = model.initSystem();
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    // initial configuration
    ground_body1->upd_coordinates(2).setValue(state, convertDegreesToRadians(q1));
    body1_body2->upd_coordinates(2).setValue(state, convertDegreesToRadians(q2));
    // define task goal
    auto x0 = fromVectorToVec3(task->x(state));
    /**
     * This is a closure (a callable function that captures the scope) that
     * implements a proportional-derivative (PD) tracking controller. The task
     * accepts a std::function which takes the state and returns a Vector.
     */
    auto pd = [&](const State& s) -> Vector { // [&] captures the current scope
        auto x = fromVectorToVec3(task->x(s));
        auto u = fromVectorToVec3(task->u(s));
        double kp = 100, kd = 20;
        auto xd = x0 + Vec3(0, 0.2 * sin(2 * Pi * s.getTime()), 0);
        auto ud = Vec3(0, 2 * Pi * 0.2 * cos(2 * Pi * s.getTime()), 0);
        auto ad = Vec3(0, -pow(2 * Pi, 2) * 0.2 * sin(2 * Pi * s.getTime()), 0);
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    task->setGoal(pd);
    //simulate
    simulate(model, state, 2);
    // export results
    forceController->printResults("ExampleAbsoluteCoordinates", ".");
    manager->printResults("ExampleAbsoluteCoordinates", ".");
    bodyKinematics->printResults("ExampleAbsoluteCoordinates", ".");
}

int main(int argc, char *argv[]) {
    try {
	absoluteCoordinates();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();e
        return -1;
    }
    return 0;
}
