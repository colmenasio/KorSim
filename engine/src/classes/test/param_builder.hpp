#pragma once

#include "model/six_dof/model.hpp"

namespace korsim::six_dof
{
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
        p.p_t = Eigen::Vector3d(-4.2, 0.0, +0.3); // m (engine thrust line, behind and slightly below CG)

        // p.wind_vel = Eigen::Vector3d(8.0, 0.0, 0.0); // m/s (just a little bit o' banter)
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

    Parameters createCommercialParameters()
    {
        Parameters p;

        // Mass properties (Heavy airliner, ~80% fuel load)
        p.mass = 65000.0; // kg (typical narrow-body airliner)

        // Moment of inertia [J_x, 0, -J_xz; 0, J_y, 0; -J_xz, 0, J_z]
        // Significantly larger due to greater mass and span
        p.moment_inertia << 280000.0, 0.0, -15000.0,
            0.0, 4500000.0, 0.0,
            -15000.0, 0.0, 650000.0; // kg·m²

        // Geometry (Larger wing and tail)
        p.S = 125.0;  // m² (wing area, ~4x fighter)
        p.S_t = 32.0; // m² (tail area, ~3.8x fighter)

        // Positions relative to CG (CG = [0,0,0])
        p.p_cg = Eigen::Vector3d(0.0, 0.0, 0.0);

        // COMMERCIAL: LARGER STATIC MARGIN
        // AC is further AFT relative to CG (CG is far forward)
        // Static margin ≈ 22% MAC vs fighter's ~10%
        p.p_ac = Eigen::Vector3d(1.8, 0.0, 0.0); // m (AC is 1.8m AFT of CG for high stability)

        p.p_t = Eigen::Vector3d(-14.5, 0.0, -0.5); // m (thrust line, far behind CG, slightly below for pitch coupling)

        p.wind_vel = Eigen::Vector3d(0.0, 0.0, 0.0); // m/s

        p.alpha_0 = -0.02;      // rad (slight negative zero-lift AOA for cruise efficiency)
        p.epsilon_alpha = 0.45; // downwash gradient (higher due to larger wing wake)

        p.air_density = 1.225; // kg/m³ (sea level)

        // Lift coefficients (1/rad)
        // COMMERCIAL: HIGHER LIFT SLOPE due to larger aspect ratio wing
        p.CL_alpha = 5.6;   // wing-body lift curve slope (higher efficiency)
        p.CL_alpha_t = 4.2; // tail lift curve slope (larger, more effective tail)
        p.CL_q_V = 8.5;     // pitch rate lift coefficient (higher due to longer fuselage)

        // Drag coefficients
        // COMMERCIAL: LOWER DRAG for efficiency, but higher induced drag due to stability
        p.CD_0 = 0.016;   // zero-lift drag (slightly lower, clean design)
        p.CD_1 = 0.010;   // offset term
        p.CD_2 = 0.52;    // quadratic coefficient (higher induced drag from tail downforce)
        p.CD_alpha = 6.8; // converts AOA to effective CD parameter

        // Sideforce coefficients (Similar to fighter, scaled for size)
        p.CY_beta = -1.1;    // sideslip stability (stronger due to larger vertical stabilizer)
        p.CY_delta_R = 0.28; // rudder effectiveness

        // Rolling moment coefficients
        p.Cl_beta = -0.120;   // dihedral effect (more stable, swept wings)
        p.Cl_p = -0.55;       // roll damping (higher inertia, slower roll)
        p.Cl_delta_A = 0.22;  // aileron effectiveness (larger control surfaces)
        p.Cl_delta_R = 0.030; // rudder-induced roll

        // Pitching moment coefficients
        // COMMERCIAL: MUCH STRONGER STABILITY AND DAMPING
        p.Cm_0 = 0.055;       // zero-AOA moment (higher trim requirement)
        p.Cm_alpha = -1.35;   // pitch stiffness (TWICE as stable as fighter, -0.72 vs -1.35)
        p.Cm_q = -18.0;       // pitch damping (much stronger, long-period phugoid damping)
        p.Cm_delta_E = -1.40; // elevator effectiveness (larger elevator, but less authority relative to mass)

        // Yawing moment coefficients
        // COMMERCIAL: STRONG DIRECTIONAL STABILITY (dutch roll heavily damped)
        p.Cn_beta = 0.18;       // directional stability (stronger yaw stability)
        p.Cn_alpha_beta = 0.06; // coupling term (slightly higher due to swept wing)
        p.Cn_p = -0.035;        // roll-yaw coupling
        p.Cn_r = -0.22;         // yaw damping (VERY strong, kills dutch roll quickly)
        p.Cn_delta_R = -0.12;   // rudder yaw effectiveness

        p.ch = 4.2;     // m (mean aerodynamic chord, longer than fighter)
        p.tau_t = 0.25; // s (thrust command delay, SLOWER response for passenger comfort)

        return p;
    }
}
