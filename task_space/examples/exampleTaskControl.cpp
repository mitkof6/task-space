#include <iostream>
#include <OpenSim/OpenSim.h>
#include "KinematicTask.h"
#include "TaskPriorityGraph.h"
#include "TaskManager.h"
#include "ConstraintProjection.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void testTaskControl() {
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

    // TaskPriorityGraph graph;
    // auto task = new PositionTask("block", Vec3(0));
    // graph.addTask(task, NULL);
    // model.addComponent(task);

    // auto constraintModel = new UnconstraintModel();
    // model.addComponent(constraintModel);
    // auto manager = new TaskManager(&graph, constraintModel);
    // model.addComponent(manager);

    auto state = model.initSystem();
    joint->updCoordinate(FreeJoint::Coord::TranslationY).setValue(state, 0.5);
    simulate(model, state, 1);
}

int main(int argc, char *argv[argc]) {
    try {
	testTaskControl();
    } catch (exception &e) {
	cout << typeid(e).name() << ": " << e.what() << endl;
	// getchar();e
	return -1;
    }
    return 0;
}
