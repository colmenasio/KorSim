#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "util/util.hpp"

namespace korsim::six_dof
{

    struct Parameters
    {
        double mass;                    // in kg
        double l_t;                     // longitudinal distance from aerodinamic center to center of tail
        Eigen::Matrix3d moment_inertia; // moment of inertia [J_x, 0, -Jxz; 0, J_y, 0; -J_xz, 0, J_z]

        double S;             // Platform area wing
        double S_t;           // Platform area tail
        Eigen::Vector3d p_cg; // Pos Center of gravity
        Eigen::Vector3d p_ac; // Pos Aerodynamic center
        Eigen::Vector3d p_t;  // Pos engine

        Eigen::Vector3d wind_vel; // wind velocity in intertia frame
        double alpha_0;           // value at which the angle of attack becomes zero
        double epsilon_alpha;     // slope constant for downwash angle

        double air_density;

        // Stability and control derivatives for lift coeff
        double CL_alpha;   // angle of attack's stability derivative
        double CL_alpha_t; // angle of attack at the tail's stability derivative
        double CL_q_V;     // q (ywise angular rate) and V (magnitude of lineal vel) stability derivative

        // Stability derivatives for drag coeff
        double CD_0;     // Drag
        double CD_1;     // Drag
        double CD_2;     // Drag
        double CD_alpha; // Drag

        // Stability and control derivatives for the sideforce coefficient
        double CY_beta;
        double CY_delta_R;

        // Stability and control derivatives for the rolling moment
        double Cl_beta;
        double Cl_p;
        double Cl_delta_A;
        double Cl_delta_R;

        // Stability and control derivatives for the pitching moment
        double Cm_0;
        double Cm_alpha;
        double Cm_q;
        double Cm_delta_E;

        // Stability and control derivatives for the yawing moment
        double Cn_beta;
        double Cn_alpha_beta;
        double Cn_p;
        double Cn_r;
        double Cn_delta_R;

        double ch; // mean aerodinamic cord

        double tau_t; // thrust command delay
    };

    struct Control
    {
        double delta_A = 0;
        double delta_E = 0;
        double delta_R = 0;
        double eta_T = 0;
    };

    struct State
    {
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d phi = Eigen::Vector3d::Zero();
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        double delta_t = 0;

        void loadMatrix(const Eigen::Matrix<double, 13, 1>& blob)
        {
            p = blob.segment<3>(0);
            v = blob.segment<3>(3);
            phi = blob.segment<3>(6);
            omega = blob.segment<3>(9);
            delta_t = blob(12);
        }

        Eigen::Matrix<double, 13, 1> toMatrix() const
        {
            Eigen::Matrix<double, 13, 1> blob;
            blob.segment<3>(0) = p;
            blob.segment<3>(3) = v;
            blob.segment<3>(6) = phi;
            blob.segment<3>(9) = omega;
            blob(12) = delta_t;
            return blob;
        }
    };

    class Model
    {
        State x_dot;
        Eigen::Vector3d F_e;
        Eigen::Vector3d F_a;
        Eigen::Vector3d F_g;

    public:
        // Calculates the dynamics and stores all results internally. Said results can be obtained with their respective getters
        void calculateDynamics(const State &x, const Control &u, const Parameters &p);

        State getDynamics() const { return x_dot; };

        Eigen::Vector3d getEngineForce() const { return F_e; };
        Eigen::Vector3d getAerodynamicForce() const { return F_a; };
        Eigen::Vector3d getGravityForce() const { return F_g; };

        // etc... en un furturo
    };

}