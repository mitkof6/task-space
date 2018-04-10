/**
 * @file ExamplePerreault2001Experiment.cpp
 *
 * \brief Control of the MoBL 2016 upper limb model.
 *
 * [1] Perreault, E. J., Kirsch, R. F., & Crago, P. E. (2001). Effects of
 * voluntary force generation on the elastic components of endpoint stiffness.
 * Experimental Brain Research, 141(3), 312–323.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <TaskSpace.h>
#include <Settings.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

Vec3 fromVectorToVec3(const Vector& v) {
    return Vec3(v[0], v[1], v[2]);
}

void findExperimentConfiguration(State& state, Model& model,
                                 double thetaShoulderDeg,
                                 double thetaElbowDeg) {
    // add orientation osensor
    SimTK::OrientationSensors* imus = new SimTK::OrientationSensors();
    auto humerusMX = imus->addOSensor(
        "humerus",
        model.updBodySet().get("humerus").getMobilizedBodyIndex(),
        SimTK::Rotation(),
        1);

    auto radiusMX = imus->addOSensor(
        "radius",
        model.updBodySet().get("radius").getMobilizedBodyIndex(),
        SimTK::Rotation(),
        1);

    // finalize observation order (to allocate ObservationIx)
    static const int OSENSORS = 2;
    static const char* osensor_observation_order[2] = {"humerus", "radius"};
    imus->defineObservationOrder(OSENSORS, osensor_observation_order);

    // get all ObservationIx
    auto humerusOX = imus->getObservationIxForOSensor(humerusMX);
    auto radiusOX = imus->getObservationIxForOSensor(radiusMX);

    // move to initial target
    SimTK::Assembler ik(model.updMultibodySystem());
    ik.setAccuracy(1e-5);
    ik.adoptAssemblyGoal(imus);
    imus->moveOneObservation(humerusOX, SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence,
        convertDegreesToRadians(-60), SimTK::XAxis,
        convertDegreesToRadians(thetaShoulderDeg), SimTK::YAxis,
        convertDegreesToRadians(0), SimTK::ZAxis));
    imus->moveOneObservation(radiusOX, SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence,
        convertDegreesToRadians(-90), SimTK::XAxis,
        convertDegreesToRadians(thetaShoulderDeg + thetaElbowDeg), SimTK::YAxis,
        convertDegreesToRadians(0), SimTK::ZAxis));

    // setup inverse kinematics
    state.setTime(0);
    ik.initialize(state);
    ik.assemble(state);
}

void perreault2001Experiment() {
    const string example = "ExamplePerreault2001Experiment";

    cout << "Warning: The model geometry may not be visible if OpenSim's " <<
        "Geometry folder is missing. This does not affect the simulation" << endl;
    // load model
    // Model model(DATA_DIR + "/mobl/mobl_2016_ideal_muscles.osim");
    Model model(DATA_DIR + "/mobl/mobl_2016_simplified_ideal_muscles.osim");
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
     * Define the control strategy \f$ \tau = J_t^T f_t $ as a callable
     *  function/closure ([&] captures the current scope). This function
     *  accepts the state and returns a Vector.
     */
    auto controlStrategy = [&](const State& s) -> Vector {
        auto data = taskDynamics->calcTaskDynamicsData(s);
        return data.tauTasks - 20 * data.NgT * s.getU();
        /*return ~handTask->J(s) * Vector(Vec3(0, 0, -100)) +
            data.NgT * (data.f + data.bc);*/
    };
    // define the controller (choose between a torque or muscle controller)
    // auto controller = new TaskBasedTorqueController(controlStrategy);
    auto controller = new TaskBasedComputedMuscleControl(controlStrategy);
    model.addController(controller);

    // build and initialize model
    auto& state = model.initSystem();
    findExperimentConfiguration(state, model, 60, 120);
    model.realizePosition(state);

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
        auto x = handTask->x(s);
        auto u = handTask->u(s);
        auto xd = handx0;
        xd(3, 3) += Vector(Vec3(0, 0, 0.1 * sin(2 * Pi * s.getTime())));
        return Vector(kp * (xd - x) - kd * u);
    };
    handTask->setGoal(handGoal);

    for (int i = 0; i < model.getCoordinateSet().getSize(); i++) {
        if (!model.getCoordinateSet()[i].isConstrained(state))
            cout << model.getCoordinateSet()[i].getName() << " "
            << convertRadiansToDegrees(model.getCoordinateSet()[i].getValue(state)) << endl;
    }

    //simulate
    simulate(model, state, 1.0, true);

    // export results
    controller->printResults(example, DATA_DIR + "/results");
    bodyKinematics->printResults(example, DATA_DIR + "/results");
}

int main(int argc, char *argv[]) {
    try {
        perreault2001Experiment();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}