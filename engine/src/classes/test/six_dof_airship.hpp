#include <Eigen/Dense>

#include <util/util.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

#include "model/six_dof/model.hpp"
#include "model/six_dof/logger.hpp"


// Forward euler pq despues de estar 8 horas programando, va a hacer runge kutta su abuela en motocicleta !!!!! :D
class SixDofAircraft : public godot::Node3D
{
    GDCLASS(SixDofAircraft, godot::Node3D);

    korsim::six_dof::Parameters parameters;
    korsim::six_dof::Control control;
    korsim::six_dof::State state;

    korsim::six_dof::Model model;

    korsim::six_dof::Logger logger;

    bool has_exploded;

    protected:
    static void _bind_methods();

    public:
	SixDofAircraft();
	~SixDofAircraft(){};

	void _process(double p_delta) override;
	void _physics_process(double p_delta) override;

    double getDeltaA() const { return control.delta_A; }
    double getDeltaR() const { return control.delta_R; }
    double getDeltaE() const { return control.delta_E; }
    double getEtaT() const { return control.eta_T; }
    godot::Vector3 getAirspaceVelocity() const { return korsim::util::posAerospaceToGodot(state.v); }
    
    void setDeltaA(double value) { control.delta_A = value; }
    void setDeltaR(double value) { control.delta_R = value; }
    void setDeltaE(double value) { control.delta_E = value; }
    void setEtaT(double value) { control.eta_T = value; }
    void setAirspaceVelocity(godot::Vector3 value) { state.v = korsim::util::posGodotToAerospace(value); }

    private:

    void integrateDynamics(double h);

};