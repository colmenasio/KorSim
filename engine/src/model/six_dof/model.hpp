#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "util/util.hpp"

namespace korsim::six_dof
{

    struct Parameters
    {
        double mass; // in kg
        double l_t; // longitudinal distance from aerodinamic center to center of tail
        Eigen::Matrix3d moment_inertia; // moment of inertia [J_x, 0, -Jxz; 0, J_y, 0; -J_xz, 0, J_z]
        
        double S; // Platform area wing
        double S_t; // Platform area tail
        Eigen::Vector3d p_cg; // Pos Center of gravity
        Eigen::Vector3d p_ac; // Pos Aerodynamic center
        Eigen::Vector3d p_t; // Pos engine

        Eigen::Vector3d wind_vel; //wind velocity in intertia frame
        double alpha_0; // value at which the angle of attack becomes zero
        double epsilon_alpha; // slope constant for downwash angle

        double air_density;

        double CL_alpha; // angle of attack's stability derivative
        double CL_alpha_t; // angle of attack at the tail's stability derivative 
        double CL_q_V; // q (ywise angular rate) and V (magnitude of lineal vel) stability derivative
        
        double CD_0; // Drag
        double CD_1; // Drag
        double CD_2; // Drag
        double CD_a; // Drag
        
        double CY_b; // Stability derivative
        double CY_delta_R; // Control derivative
        
        double Cl_beta;
        double Cl_p;
        double Cl_delta_A;
        double Cl_delta_R;

        double Cm_0;
        double Cm_alpha;
        double Cm_q;
        double Cm_delta_E;

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
        double delta_A;
        double delta_E;
        double delta_R;
        double eta_t;
    };

    struct State
    {
        Eigen::Vector3d p; 
        Eigen::Vector3d v; 
        Eigen::Vector3d phi; 
        Eigen::Vector3d omega; 
        double delta_t;
    };

    class Model
    {
        State x_dot;

    public:

        // Calculates the dynamics and stores all results internally. Said results can be obtained with their respective getters
        void calculateDynamics(const State& x, const Control& u, const Parameters& p);


    private:
    };

}