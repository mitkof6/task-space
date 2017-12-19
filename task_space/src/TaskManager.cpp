#include "TaskManager.h"
#include "TaskPriorityGraph.h"
#include "TaskProjection.h"
#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

TaskManager::TaskManager(std::shared_ptr<TaskPriorityGraph> graph)
    : taskPriorityGraph(graph) {

}

Vector TaskManager::calcTaskTorques(const State& s) {

    Matrix MInv = calcMInv(s, *_model);
    Vector f = calcTotalGeneralizedForces(s, *_model);
    Matrix NT(s.getNU(), s.getNU()); NT = 1;
    Vector tauP(s.getNU(), 0.0);


    return Vector();
}
