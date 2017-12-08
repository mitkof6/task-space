#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void test() {

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
