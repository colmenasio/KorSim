#pragma once

#include <Eigen/Dense>
#include <godot_cpp/variant/vector3.hpp>

namespace korsim::util
{
    inline Eigen::Matrix3d rotationFromEuler(const Eigen::Vector3d &roll_pitch_yaw)
    {
        double yaw = roll_pitch_yaw(0);
        double pitch = roll_pitch_yaw(1);
        double roll = roll_pitch_yaw(2);

        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);

        // Tipica matriz que es coñazo de debuggear...
        Eigen::Matrix3d R;

        R(0, 0) = cy * cp;
        R(0, 1) = cy * sp * sr - sy * cr;
        R(0, 2) = cy * sp * cr + sy * sr;

        R(1, 0) = sy * cp;
        R(1, 1) = sy * sp * sr + cy * cr;
        R(1, 2) = sy * sp * cr - cy * sr;

        R(2, 0) = -sp;
        R(2, 1) = cp * sr;
        R(2, 2) = cp * cr;

        return R;
    }

    inline Eigen::Matrix3d inverseRotationFromEuler(const Eigen::Vector3d &roll_pitch_yaw)
    {
        return rotationFromEuler(roll_pitch_yaw).transpose(); // Ortonormal :3
    }

    // Aerospace ned : X=North, Y=East, Z=Down
    // Godot: X=East, Y=Up, Z=South
    inline godot::Vector3 posAerospaceToGodot(const Eigen::Vector3d &pos)
    {
        return godot::Vector3(pos.y(), -pos.z(), -pos.x());
    }

    inline Eigen::Vector3d posGodotToAerospace(const godot::Vector3 &pos)
    {
        return Eigen::Vector3d(-pos.z, pos.x, -pos.y);
    }

    inline godot::Vector3 eulerAerospaceToGodot(const Eigen::Vector3d &roll_pitch_yaw)
    {
        //return godot::Vector3(roll_pitch_yaw.x(), -roll_pitch_yaw.z(), roll_pitch_yaw.y());
        return godot::Vector3(roll_pitch_yaw.y(), -roll_pitch_yaw.z(), -roll_pitch_yaw.x());
    }

    inline Eigen::Vector3d eulerGodotToAerospace(const godot::Vector3 &roll_pitch_yaw)
    {
        return Eigen::Vector3d(-roll_pitch_yaw.z, roll_pitch_yaw.x, -roll_pitch_yaw.y);
    }
};