/**
 * \file This example demonstrating working example of controlling (task space)
 * a model of a closed kinematic chain topology that is build using absolute
 * (Cartesian) coordinates. Since the underlying dynamics use constraint
 * projection, constraints are implicitly accounted. For more details please
 * refer to Section Supplementary Material(E).
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

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(&v(0, 3)[0]);
}

void closedKinematicChain() {
    string taskBody = "body1";
    const double q1 = -0.0;
    double body3_length = 1;
    // model
    Model model;
    // configure the visualizer
    model.setUseVisualizer(true);
    // construct model
    auto& ground = model.getGround();
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
    auto ground_body1 = new OpenSim::PinJoint("ground_body1", ground,
                                              Vec3(0), Vec3(0),
                                              *body1_body, body1_distal, Vec3(0));
    ground_body1->upd_coordinates(0)
        .setDefaultValue(SimTK::convertDegreesToRadians(q1));
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
    auto ground_body2 = new OpenSim::PinJoint("body1_body2", ground,
                                              Vec3(body3_length, 0, 0), Vec3(0),
                                              *body2_body, body2_distal, Vec3(0));
    ground_body2->upd_coordinates(0)
        .setDefaultValue(SimTK::convertDegreesToRadians(q1));
    model.addBody(body2_body);
    model.addJoint(ground_body2);
    // body3
    double body3_m = 1, body3_radius = 0.03;
    Vec3 body3_com = Vec3(0);
    Inertia body3_I = body3_m * Inertia::cylinderAlongY(
        body3_radius, body3_length);
    auto body3_body = new OpenSim::Body("body3", body3_m, body3_com, body3_I);
    auto body3_geom = new OpenSim::Cylinder(body3_radius, body3_length / 2);
    body3_geom->setName("body3_cylinder");
    body3_body->attachGeometry(body3_geom);
    Vec3 body3_distal(0, -body3_length / 2, 0);
    Vec3 body3_proximal(0, body3_length / 2, 0);
    auto ground_body3 = new OpenSim::FreeJoint("body1_body3", ground,
                                               Vec3(0), Vec3(0), *body3_body,
                                               body3_distal, Vec3(0));
    model.addBody(body3_body);
    model.addJoint(ground_body3);
    // connect two free bodies
    auto pointConstraint1 = new PointConstraint(*body1_body, body1_proximal,
                                                *body3_body, body3_distal);
    model.addConstraint(pointConstraint1);
    auto pointConstraint2 = new PointConstraint(*body2_body, body2_proximal,
                                                *body3_body, body3_proximal);
    model.addConstraint(pointConstraint2);
    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);
    // construct task priority graph
    TaskPriorityGraph graph;
    auto task = new OrientationTask(taskBody, Vec3(0)); // upper body
    graph.addTask(task, NULL);
    model.addComponent(task);
    // chose constraint model
    auto constraintModel = new AghiliModel();
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
	return data.tauTasks + data.NgT * (data.f + data.bc) ;
    };
    // construct a torque controller and supply the control strategy
    auto controller = new TaskBasedTorqueController(controlStrategy);
    model.addController(controller);
    // *************************************************************************
    // build and initialize model
    auto state = model.initSystem();
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    // initial configuration
    model.updCoordinateSet()[0].setValue(state, convertDegreesToRadians(q1));
    model.updCoordinateSet()[1].setValue(state, convertDegreesToRadians(q1));
    // define task goal
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
        auto xd = x0 + Vec3(0, 0, Pi / 2 * sin(2 * Pi * s.getTime()));
        auto ud = Vec3(0, 0, 2 * Pi * Pi / 2 * cos(2 * Pi * s.getTime()));
        auto ad = Vec3(0, 0, -pow(2 * Pi, 2) * Pi / 2 * sin(2 * Pi * s.getTime()));
        return Vector(ad + kp * (xd - x) + kd * (ud - u));
    };
    task->setGoal(pd);
    //simulate
    simulate(model, state, 2);
    // export results
    controller->printResults("ExampleClosedKinematicChain", ".");
    bodyKinematics->printResults("ExampleClosedKinematicChain", ".");
}

int main(int argc, char *argv[]) {
    try {
        closedKinematicChain();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();e
        return -1;
    }
    return 0;
}
