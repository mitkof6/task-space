#include "KinematicTask.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace SimTK;

/** Concatenates two Vec3 into Vector */
Vector toVector(const Vec3& o, const Vec3& p) {
    Vector spatial(6, 0.0);
    spatial[0] = o[0];
    spatial[1] = o[1];
    spatial[2] = o[2];
    spatial[3] = p[0];
    spatial[4] = p[1];
    spatial[5] = p[2];
    return spatial;
}

/** Concatenates a SpatialVec into Vector */
Vector spatialToVector(const SpatialVec& spatial) {
    Vec3 o = spatial[0];
    Vec3 p = spatial[1];
    Vector vec(6, 0.0);
    vec[0] = o[0];
    vec[1] = o[1];
    vec[2] = o[2];
    vec[3] = p[0];
    vec[4] = p[1];
    vec[5] = p[2];
    return vec;
}

/******************************************************************************/

KinematicTask::KinematicTask(std::string body, Vec3 offset)
    : body(body), offset(offset) {
}

void KinematicTask::setGoal(const TaskGoal& g) {
    goal = g;
}

SimTK::Vector KinematicTask::getGoal(const State& s) const {
    return goal(s);
}

/******************************************************************************/

PositionTask::PositionTask(std::string body, Vec3 offset)
    : KinematicTask(body, offset) {
}

Matrix PositionTask::J(const State& s) const {
    Matrix J;
    _model->getMatterSubsystem()
        .calcStationJacobian(s,
                             _model->getBodySet().get(body)
                             .getMobilizedBodyIndex(), offset, J);
    return J;
}

Vector PositionTask::b(const State& s) const {
    Vec3 JdotQdot = _model->getMatterSubsystem().
        calcBiasForStationJacobian(
        s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset);
    return Vector(-1.0 * JdotQdot);
}

Vector PositionTask::x(const State& s) const {
    Vec3 temp;
    _model->getSimbodyEngine()
        .getPosition(s, _model->getBodySet().get(body), offset, temp);
    return Vector(temp);
}

Vector PositionTask::u(const State& s) const {
    Vec3 temp;
    _model->getSimbodyEngine()
        .getVelocity(s, _model->getBodySet().get(body), offset, temp);
    return Vector(temp);
}

Vector PositionTask::a(const State& s) const {
    Vec3 temp;
    _model->getSimbodyEngine()
        .getAcceleration(s, _model->getBodySet().get(body), offset, temp);
    return Vector(temp);
}

/******************************************************************************/

OrientationTask::OrientationTask(std::string body, Vec3 offset)
    : KinematicTask(body, offset) {
}

Matrix OrientationTask::J(const State& s) const {
    Matrix J;
    _model->getMatterSubsystem().calcFrameJacobian(
        s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset, J);
    return J(0, 0, 3, J.ncol());
}

Vector OrientationTask::b(const State& s) const {
    SpatialVec jdu = _model->getMatterSubsystem().calcBiasForFrameJacobian(
        s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset);
    return Vector(-1.0 * jdu[0]);
}

Vector OrientationTask::x(const State& s) const {
    double dirCos[3][3];
    Vec3 o;
    _model->getSimbodyEngine()
        .getDirectionCosines(s, _model->getBodySet().get(body), dirCos);
    _model->getSimbodyEngine()
        .convertDirectionCosinesToAngles(dirCos, &o[0], &o[1], &o[2]);
    return Vector(o);
}

Vector OrientationTask::u(const State& s) const {
    Vec3 w;
    _model->getSimbodyEngine()
        .getAngularVelocity(s, _model->getBodySet().get(body), w);
    return Vector(w);
}

Vector OrientationTask::a(const State& s) const {
    Vec3 aw;
    _model->getSimbodyEngine()
        .getAngularAcceleration(s, _model->getBodySet().get(body), aw);
    return Vector(aw);
}

/******************************************************************************/

SpatialTask::SpatialTask(std::string body, Vec3 offset)
    : KinematicTask(body, offset) {
}

Matrix SpatialTask::J(const State& s) const {
    Matrix J;
    _model->getMatterSubsystem().calcFrameJacobian(
        s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset, J);
    return J;
}

Vector SpatialTask::b(const State& s) const {
    SpatialVec jdu = _model->getMatterSubsystem().calcBiasForFrameJacobian(
        s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset);
    return -1.0 * spatialToVector(jdu);
}

Vector SpatialTask::x(const State& s) const {
    double dirCos[3][3];
    Vec3 o, p;
    _model->getSimbodyEngine()
        .getPosition(s, _model->getBodySet().get(body), offset, p);
    _model->getSimbodyEngine()
        .getDirectionCosines(s, _model->getBodySet().get(body), dirCos);
    _model->getSimbodyEngine()
        .convertDirectionCosinesToAngles(dirCos, &o[0], &o[1], &o[2]);
    return toVector(o, p);
}

Vector SpatialTask::u(const State& s) const {
    Vec3 w, v;
    _model->getSimbodyEngine()
        .getVelocity(s, _model->getBodySet().get(body), offset, v);
    _model->getSimbodyEngine()
        .getAngularVelocity(s, _model->getBodySet().get(body), w);
    return toVector(w, v);
}

Vector SpatialTask::a(const State& s) const {
    Vec3 aw, a;
    _model->getSimbodyEngine()
        .getAcceleration(s, _model->getBodySet().get(body), offset, a);
    _model->getSimbodyEngine()
        .getAngularAcceleration(s, _model->getBodySet().get(body), aw);
    return toVector(aw, a);
}
