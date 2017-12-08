#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include "KinematicTask.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void test() {
    KinematicTask* task = new PositionTask("pelvis", Vec3(0));
    // TODO
}

int main(int argc, char *argv[argc]) {
    try {
	test();
    } catch (exception &e) {
	cout << e.what() << endl;
	// getchar();
	return -1;
    }
    return 0;
}
