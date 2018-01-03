/**
 * @file TestTaskPriorityGraph.cpp
 *
 *\brif Tests the behavior of the TaskPriorityGraph when adding a prioritized
 * task.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include <iostream>
#include <typeinfo>
#include <KinematicTask.h>
#include <TaskDynamics.h>
#include "TestUtil.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void testGraphConstruction() {
    TaskDynamics dynamics(NULL);
    auto pelvis = new SpatialTask("pelvis", Vec3(0));
    dynamics.addTask(pelvis, NULL);
    auto femur = new OrientationTask("femur", Vec3(0));
    dynamics.addTask(femur, pelvis);
    auto tibia = new OrientationTask("tibia", Vec3(0));
    dynamics.addTask(tibia, femur);
    auto torso = new OrientationTask("torso", Vec3(0));
    dynamics.addTask(torso, pelvis);
    cout << dynamics << endl;
    // check singularities
    MUST_THROW_EXCEPTION(dynamics.addTask(pelvis, NULL),
                         TaskExistsInGraphException);
    auto ground = new PositionTask("ground", Vec3(0));
    MUST_THROW_EXCEPTION(dynamics.addTask(ground, ground),
                         ParentNotInGraphException);
}

int main(int argc, char *argv[]) {
    try {
        testGraphConstruction();
    } catch (exception &e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}