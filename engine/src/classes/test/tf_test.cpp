
//gemini maxxing
// In your Node3D class implementation (e.g., TfTestNode3D.cpp)

#include "tf_test.hpp"

void TfTestNode3D::_bind_methods() {
    // Bind methods
    godot::ClassDB::bind_method(godot::D_METHOD("set_pos_aero", "val"), &TfTestNode3D::setPosAero);
    godot::ClassDB::bind_method(godot::D_METHOD("get_pos_aero"), &TfTestNode3D::getPosAero);
    godot::ClassDB::bind_method(godot::D_METHOD("set_euler_aero", "val"), &TfTestNode3D::setEulerAero);
    godot::ClassDB::bind_method(godot::D_METHOD("get_euler_aero"), &TfTestNode3D::getEulerAero);
    
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "pos_aero"), "set_pos_aero", "get_pos_aero");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "euler_aero"), "set_euler_aero", "get_euler_aero");
}

void TfTestNode3D::setPosAero(godot::Vector3 val) {
    Eigen::Vector3d eigenPos(val.x, val.y, val.z);
    godot::Vector3 godotPos = posAerospaceToGodot(eigenPos);
    set_position(godotPos);
}

godot::Vector3 TfTestNode3D::getPosAero() const {
    godot::Vector3 godotEuler = get_position();
    Eigen::Vector3d eigenEuler = posGodotToAerospace(godotEuler);
    return godot::Vector3(eigenEuler.x(), eigenEuler.y(), eigenEuler.z());
}

void TfTestNode3D::setEulerAero(godot::Vector3 val) {
    Eigen::Vector3d eigenNEDEuler(val.x, val.y, val.z);
    godot::Vector3 godotEuler = eulerAerospaceToGodot(eigenNEDEuler);
    set_rotation(godotEuler);
}

godot::Vector3 TfTestNode3D::getEulerAero() const {
    godot::Vector3 godotEuler = get_rotation();
    Eigen::Vector3d eigenEuler = eulerGodotToAerospace(godotEuler);
    return godot::Vector3(eigenEuler.x(), eigenEuler.y(), eigenEuler.z());
}