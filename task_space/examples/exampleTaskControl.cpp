#include <iostream>
#include <typeinfo>
#include <OpenSim/Simulation/Model/Model.h>
#include "KinematicTask.h"
#include "TaskPriorityGraph.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void testTaskControl() {
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
