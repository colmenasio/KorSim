#pragma once

#include <Eigen/Dense>
#include <godot_cpp/variant/vector3.hpp>
#include <cmath>

namespace korsim::util
{
    inline Eigen::Matrix3d rotationFromRoll(double roll)
    {
        Eigen::Matrix3d R;
        double c = std::cos(roll);
        double s = std::sin(roll);
        
        R << 1, 0, 0,
            0, c, -s,
            0, s, c;
        return R;
    }

    inline Eigen::Matrix3d rotationFromPitch(double pitch)
    {
        Eigen::Matrix3d R;
        double c = std::cos(pitch);
        double s = std::sin(pitch);

        R << c, 0, s,
            0, 1, 0,
            -s, 0, c;
        return R;
    }

    inline Eigen::Matrix3d rotationFromYaw(double yaw)
    {
        Eigen::Matrix3d R;
        double c = std::cos(yaw);
        double s = std::sin(yaw);

        R << c, -s, 0,
            s, c, 0,
            0, 0, 1;
        return R;
    }

    inline Eigen::Matrix3d rotationFromEuler(const Eigen::Vector3d &roll_pitch_yaw)
    {
        double roll = roll_pitch_yaw(0);
        double pitch = roll_pitch_yaw(1);
        double yaw = roll_pitch_yaw(2);

        Eigen::Matrix3d R_r = rotationFromRoll(roll);
        Eigen::Matrix3d R_p = rotationFromPitch(pitch);
        Eigen::Matrix3d R_y = rotationFromYaw(yaw);
        return R_y * R_p * R_r;
    }

    inline Eigen::Matrix3d inverseRotationFromEuler(const Eigen::Vector3d &roll_pitch_yaw)
    {
        return rotationFromEuler(roll_pitch_yaw).transpose(); // Ortonormal :3
    }

    inline Eigen::Matrix3d inverseKinematicJacobian(const Eigen::Vector3d &roll_pitch_yaw, double epsilon = 1e-4)
    {
        double roll = roll_pitch_yaw(0);
        double pitch = roll_pitch_yaw(1);

        double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);

        // gimball i curse you!!
        if (std::abs(cp) < epsilon)
        {
            if (cp >= 0)
            {
                cp = epsilon;
            }
            else
            {
                cp = -epsilon;
            }
        }

        const double tp = sp / cp;

        // Tipica matriz que es coñazo de debuggear...
        Eigen::Matrix3d R;

        R(0, 0) = 1;
        R(0, 1) = sr * tp;
        R(0, 2) = cr * tp;

        R(1, 0) = 0;
        R(1, 1) = cr;
        R(1, 2) = -sr;

        R(2, 0) = 0;
        R(2, 1) = sr / cp;
        R(2, 2) = cr / cp;

        return R;
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
        return godot::Vector3(roll_pitch_yaw.y(), -roll_pitch_yaw.z(), -roll_pitch_yaw.x()); //* 180 * M_1_PI;
    }

    inline Eigen::Vector3d eulerGodotToAerospace(const godot::Vector3 &roll_pitch_yaw)
    {
        return Eigen::Vector3d(-roll_pitch_yaw.z, roll_pitch_yaw.x, -roll_pitch_yaw.y); //* M_PI * 0.00555555555;
    }
};