/**
 * \file Tests the behavior of the TaskPriorityGraph when adding a prioritized
 * task.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <iostream>
#include <typeinfo>
#include <OpenSim/Simulation/Model/Model.h>
#include "KinematicTask.h"
#include "TaskPriorityGraph.h"
#include "TestUtil.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void testGraphConstruction() {
    TaskPriorityGraph graph;
    auto pelvis = new SpatialTask("pelvis", Vec3(0));
    graph.addTask(pelvis, NULL);
    auto femur = new OrientationTask("femur", Vec3(0));
    graph.addTask(femur, pelvis);
    auto tibia = new OrientationTask("tibia", Vec3(0));
    graph.addTask(tibia, femur);
    auto torso = new OrientationTask("torso", Vec3(0));
    graph.addTask(torso, pelvis);
    cout << graph << endl;
    // check singularities
    MUST_THROW_EXCEPTION(graph.addTask(pelvis, NULL), TaskExistsInGraphException);
    auto ground = new PositionTask("ground", Vec3(0));
    MUST_THROW_EXCEPTION(graph.addTask(ground, ground), ParentNotInGraphException);
}

int main(int argc, char *argv[]) {
    try {
	testGraphConstruction();
    } catch (exception &e) {
	cout << typeid(e).name() << ": " << e.what() << endl;
	// getchar();e
	return -1;
    }
    return 0;
}
