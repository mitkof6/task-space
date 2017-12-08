#include "KinematicTask.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

KinematicTask::KinematicTask(std::string body, SimTK::Vec3 offset)
    : body(body), offset(offset)
{
}

PositionTask::PositionTask(std::string body, SimTK::Vec3 offset)
    : KinematicTask(body, offset)
{
}

SimTK::Matrix PositionTask::J(const SimTK::State& s) const {

    SimTK::Matrix J;
    _model->getMatterSubsystem()
	.calcStationJacobian(s,
			     _model->get_BodySet().get(body)
			     .getMobilizedBodyIndex(), offset, J);
    return J;
}

SimTK::Vector PositionTask::b(const SimTK::State& s) const {

    SimTK::Vec3 JdotQdot = _model->getMatterSubsystem().
	calcBiasForStationJacobian(
	    s, _model->getBodySet().get(body).getMobilizedBodyIndex(), offset);
    return SimTK::Vector(-1.0 * JdotQdot);
}

SimTK::Vector PositionTask::x(const SimTK::State& s) const {
    SimTK::Vec3 temp;
    _model->getSimbodyEngine().getPosition(s,
	_model->getBodySet().get(body), offset, temp);
    return SimTK::Vector(temp);
}

SimTK::Vector PositionTask::u(const SimTK::State& s) const {
    SimTK::Vec3 temp;
    _model->getSimbodyEngine().getVelocity(s,
	_model->getBodySet().get(body), offset, temp);
    return SimTK::Vector(temp);
}

SimTK::Vector PositionTask::a(const SimTK::State& s) const {
    SimTK::Vec3 temp;
    _model->getSimbodyEngine().getAcceleration(s,
	_model->getBodySet().get(body), offset, temp);
    return SimTK::Vector(temp);
}
