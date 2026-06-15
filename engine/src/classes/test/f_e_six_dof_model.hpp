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
	~FESixDofModel(){};

    void computeDynamics(double h);
    void computeAndIntegrateDynamics(double h);

    double getDeltaA() const { return control.delta_A; }
    double getDeltaR() const { return control.delta_R; }
    double getDeltaE() const { return control.delta_E; }
    double getEtaT() const { return control.eta_T; }
    godot::Vector3 getAirspaceVelocity() const { return korsim::util::posAerospaceToGodot(state.v); }
    godot::Vector3 getWindVelocity() const { return korsim::util::posAerospaceToGodot(parameters.wind_vel); }
    godot::Vector3 getPosition() const { return korsim::util::posAerospaceToGodot(state.p); }
    godot::Vector3 getRotation() const { return korsim::util::eulerAerospaceToGodot(state.phi); }
    godot::Vector3 getAerodynamicForce() const { return korsim::util::posAerospaceToGodot(model.getAerodynamicForce()); }
    godot::Vector3 getGravityForce() const { return korsim::util::posAerospaceToGodot(model.getGravityForce()); }
    godot::Vector3 getEngineForce() const { return korsim::util::posAerospaceToGodot(model.getEngineForce()); }
    
    void setDeltaA(double value) { control.delta_A = value; }
    void setDeltaR(double value) { control.delta_R = value; }
    void setDeltaE(double value) { control.delta_E = value; }
    void setEtaT(double value) { control.eta_T = value; }
    void setPosition(godot::Vector3 value) { state.p = korsim::util::posGodotToAerospace(value); }
    void setRotation(godot::Vector3 value) { state.phi = korsim::util::eulerGodotToAerospace(value); }
    void setAirspaceVelocity(godot::Vector3 value) { state.v = korsim::util::posGodotToAerospace(value); }
    void setWindVelocity(godot::Vector3 value) { parameters.wind_vel = korsim::util::posGodotToAerospace(value); }
};