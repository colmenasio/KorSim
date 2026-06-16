#pragma once

#include <Eigen/Dense>

#include <util/util.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

#include "model/six_dof/model.hpp"
#include "model/six_dof/logger.hpp"

// Forward euler pq despues de estar 8 horas programando, va a hacer runge kutta su abuela en motocicleta !!!!! :D
class FESixDofModel : public godot::RefCounted
{
    GDCLASS(FESixDofModel, godot::RefCounted);

    korsim::six_dof::Parameters parameters;
    korsim::six_dof::Control control;
    korsim::six_dof::State state;

    korsim::six_dof::Model model;

    korsim::six_dof::Logger logger;

protected:
    static void _bind_methods();

public:
    FESixDofModel();
    ~FESixDofModel() {};

    void computeDynamics(double h);
    void computeAndIntegrateDynamics(double h);

    double getDeltaA() const { return control.delta_A; }
    double getDeltaR() const { return control.delta_R; }
    double getDeltaE() const { return control.delta_E; }
    double getEtaT() const { return control.eta_T; }
    double getAngleOfAttack() const { return model.getAngleOfAttack(); }
    double getAngleOfAttackTail() const { return model.getAngleOfAttackTail(); }
    double getSideslipAngle() const { return model.getSideslipAngle(); }
    godot::Vector3 getAirspaceVelocity() const { return korsim::util::posAerospaceToGodot(state.v); }
    godot::Vector3 getWindVelocity() const { return korsim::util::posAerospaceToGodot(parameters.wind_vel); }
    godot::Vector3 getPosition() const { return korsim::util::posAerospaceToGodot(state.p); }
    godot::Vector3 getRotation() const { return korsim::util::eulerAerospaceToGodot(state.phi); }
    godot::Vector3 getLiftForce() const { return korsim::util::posAerospaceToGodot(model.getLiftForce()); }
    godot::Vector3 getDragForce() const { return korsim::util::posAerospaceToGodot(model.getDragForce()); }
    godot::Vector3 getSideForce() const { return korsim::util::posAerospaceToGodot(model.getSideForce()); }
    godot::Vector3 getAerodynamicForce() const { return korsim::util::posAerospaceToGodot(model.getAerodynamicForce()); }
    godot::Vector3 getGravityForce() const { return korsim::util::posAerospaceToGodot(model.getGravityForce()); }
    godot::Vector3 getEngineForce() const { return korsim::util::posAerospaceToGodot(model.getEngineForce()); }

    void setDeltaA(double value) { control.delta_A = godot::CLAMP(value, -1.0, 1.0); }
    void setDeltaR(double value) { control.delta_R = godot::CLAMP(value, -1.0, 1.0); }
    void setDeltaE(double value) { control.delta_E = godot::CLAMP(value, -1.0, 1.0); }
    void setEtaT(double value) { control.eta_T = godot::CLAMP(value, 0.0, 1.0); }
    void setPosition(godot::Vector3 value) { state.p = korsim::util::posGodotToAerospace(value); }
    void setRotation(godot::Vector3 value) { state.phi = korsim::util::eulerGodotToAerospace(value); }
    void setAirspaceVelocity(godot::Vector3 value) { state.v = korsim::util::posGodotToAerospace(value); }
    void setWindVelocity(godot::Vector3 value) { parameters.wind_vel = korsim::util::posGodotToAerospace(value); }
};