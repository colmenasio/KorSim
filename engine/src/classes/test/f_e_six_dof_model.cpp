#include "f_e_six_dof_model.hpp"
#include "param_builder.hpp"

using namespace korsim::six_dof;
using namespace korsim;
using namespace godot;

void FESixDofModel::_bind_methods()
{ 
    ClassDB::bind_method(D_METHOD("compute_dynamics", "delta"), &FESixDofModel::computeDynamics);
    ClassDB::bind_method(D_METHOD("compute_and_integrate_dynamics"), &FESixDofModel::computeAndIntegrateDynamics);

    ClassDB::bind_method(D_METHOD("set_aileron_deflector", "value"), &FESixDofModel::setDeltaA);
    ClassDB::bind_method(D_METHOD("get_aileron_deflector"), &FESixDofModel::getDeltaA);

    ClassDB::bind_method(D_METHOD("set_rudder_deflector", "value"), &FESixDofModel::setDeltaR);
    ClassDB::bind_method(D_METHOD("get_rudder_deflector"), &FESixDofModel::getDeltaR);

    ClassDB::bind_method(D_METHOD("set_elevator_deflector", "value"), &FESixDofModel::setDeltaE);
    ClassDB::bind_method(D_METHOD("get_elevator_deflector"), &FESixDofModel::getDeltaE);

    ClassDB::bind_method(D_METHOD("set_throttle", "value"), &FESixDofModel::setEtaT);
    ClassDB::bind_method(D_METHOD("get_throttle"), &FESixDofModel::getEtaT);

    ClassDB::bind_method(D_METHOD("set_position", "value"), &FESixDofModel::setPosition);
    ClassDB::bind_method(D_METHOD("get_position"), &FESixDofModel::getPosition);
    
    ClassDB::bind_method(D_METHOD("set_rotation", "value"), &FESixDofModel::setRotation);
    ClassDB::bind_method(D_METHOD("get_rotation"), &FESixDofModel::getRotation);
    
    ClassDB::bind_method(D_METHOD("set_airspace_velocity", "value"), &FESixDofModel::setAirspaceVelocity);
    ClassDB::bind_method(D_METHOD("get_airspace_velocity"), &FESixDofModel::getAirspaceVelocity);

    ClassDB::bind_method(D_METHOD("set_wind_velocity", "value"), &FESixDofModel::setWindVelocity);
    ClassDB::bind_method(D_METHOD("get_wind_velocity"), &FESixDofModel::getWindVelocity);

    ClassDB::bind_method(D_METHOD("get_angle_of_attack"), &FESixDofModel::getAngleOfAttack);
    ClassDB::bind_method(D_METHOD("get_angle_of_attack_tail"), &FESixDofModel::getAngleOfAttackTail);
    ClassDB::bind_method(D_METHOD("get_sideslip_angle"), &FESixDofModel::getSideslipAngle);
    ClassDB::bind_method(D_METHOD("get_lift_force"), &FESixDofModel::getLiftForce);
    ClassDB::bind_method(D_METHOD("get_drag_force"), &FESixDofModel::getDragForce);
    ClassDB::bind_method(D_METHOD("get_side_force"), &FESixDofModel::getSideForce);
    ClassDB::bind_method(D_METHOD("get_aerodynamic_force"), &FESixDofModel::getAerodynamicForce);
    ClassDB::bind_method(D_METHOD("get_engine_force"), &FESixDofModel::getEngineForce);
    ClassDB::bind_method(D_METHOD("get_gravity_force"), &FESixDofModel::getGravityForce);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "aileron_deflector", PROPERTY_HINT_RANGE, "-1.0,1.0,0.01"), "set_aileron_deflector", "get_aileron_deflector");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rudder_deflector", PROPERTY_HINT_RANGE, "-1.0,1.0,0.01"), "set_rudder_deflector", "get_rudder_deflector");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "elevator_deflector", PROPERTY_HINT_RANGE, "-1.0,1.0,0.01"), "set_elevator_deflector", "get_elevator_deflector");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"), "set_throttle", "get_throttle");
    //ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "airspace_velocity"), "set_airspace_velocity", "get_airspace_velocity");
    //ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "wind_velocity"), "set_airspace_velocity", "get_airspace_velocity");
}

FESixDofModel::FESixDofModel() : logger("log.last")
{
    this->parameters = createFighterParameters();
}

void FESixDofModel::computeAndIntegrateDynamics(double h)
{
    // what the fucc is runge kuttaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
    // forward euler foreverrrrrrrrrrrrrrrrrrrrrrr
    // rawrrrrrrrrrrrrrrrrrrrrrrrr
    this->computeDynamics(h);

    logger.log("---------[State]----------\n");
    logger.log(state);

    logger.log("---------[Dynamics]----------\n");
    logger.log(model);

    Eigen::Matrix<double, 13, 1> x = this->state.toMatrix();
    Eigen::Matrix<double, 13, 1> x_dot = this->model.getDynamics().toMatrix();
    this->state.loadMatrix(x + h * x_dot);
}

void FESixDofModel::computeDynamics(double h)
{
    model.calculateDynamics(state, control, parameters); 
}
