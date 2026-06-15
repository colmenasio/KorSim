#include "f_e_six_dof_model.hpp"

using namespace korsim::six_dof;
using namespace korsim;
using namespace godot;

Parameters createFighterParameters()
{
    Parameters p;

    // Mass properties
    p.mass = 12000.0; // kg (light fighter, ~50% fuel)
    p.l_t = 4.8;      // m (tail moment arm from CG)

    // Moment of inertia [J_x, 0, -J_xz; 0, J_y, 0; -J_xz, 0, J_z]
    p.moment_inertia << 50000.0, 0.0, -5000.0,
        0.0, 150000.0, 0.0,
        -5000.0, 0.0, 180000.0; // kg·m²

    // Geometry
    p.S = 32.0;  // m² (wing area)
    p.S_t = 8.5; // m² (tail area)

    // Positions relative to CG (CG = [0,0,0])
    p.p_cg = Eigen::Vector3d(0.0, 0.0, 0.0);
    p.p_ac = Eigen::Vector3d(0.45, 0.0, 0.0); // m (aerodynamic center, 0.45m ahead of CG for static margin)
    p.p_t = Eigen::Vector3d(-4.2, 0.0, -0.3); // m (engine thrust line, behind and slightly below CG)

    //p.wind_vel = Eigen::Vector3d(8.0, 0.0, 0.0); // m/s (just a little bit o' banter)
    p.wind_vel = Eigen::Vector3d(0.0, 0.0, 0.0); // m/s (just a little bit o' banter)

    p.alpha_0 = 0.0;        // rad (zero-lift AOA)
    p.epsilon_alpha = 0.35; // downwash gradient

    p.air_density = 1.225; // kg/m³ (sea level)

    // Lift coefficients (1/rad)
    p.CL_alpha = 4.8;   // wing-body lift curve slope
    p.CL_alpha_t = 3.6; // tail lift curve slope (includes downwash effects implicitly)
    p.CL_q_V = 5.2;     // pitch rate lift coefficient

    // Drag coefficients
    p.CD_0 = 0.018;   // zero-lift drag
    p.CD_1 = 0.015;   // offset term
    p.CD_2 = 0.45;    // quadratic coefficient
    p.CD_alpha = 5.2; // converts AOA to effective CD parameter

    // Sideforce coefficients
    p.CY_beta = -0.85;   // sideslip stability (negative = stable)
    p.CY_delta_R = 0.22; // rudder effectiveness

    // Rolling moment coefficients
    p.Cl_beta = -0.095;   // dihedral effect (negative = stable)
    p.Cl_p = -0.42;       // roll damping
    p.Cl_delta_A = 0.18;  // aileron effectiveness
    p.Cl_delta_R = 0.025; // rudder-induced roll

    // Pitching moment coefficients
    p.Cm_0 = 0.045;       // zero-AOA moment (trim)
    p.Cm_alpha = -0.72;   // pitch stiffness (negative = stable)
    p.Cm_q = -11.5;       // pitch damping
    p.Cm_delta_E = -1.15; // elevator effectiveness

    // Yawing moment coefficients
    p.Cn_beta = 0.11;       // directional stability (positive = stable)
    p.Cn_alpha_beta = 0.04; // coupling term
    p.Cn_p = -0.028;        // roll-yaw coupling
    p.Cn_r = -0.13;         // yaw damping (negative = dutch roll damping)
    p.Cn_delta_R = -0.09;   // rudder yaw effectiveness

    p.ch = 3.4;     // m (mean aerodynamic chord)
    p.tau_t = 0.08; // s (thrust command delay, faster for fighter)

    return p;
}

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
