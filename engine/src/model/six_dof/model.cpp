#include "model.hpp"

using namespace Eigen;
using namespace korsim::util;

#include <iostream>

inline double square(double x){
    return x*x;
} 

void korsim::six_dof::Model::calculateDynamics(const State& x, const Control& u, const Parameters& p)
{
    Matrix3d R_i2b = rotationFromEuler(x.phi);
    Matrix3d R_b2i = R_i2b.transpose();

    const double g = 9.8;

    double vel_mod = x.v.norm();
    double alpha = std::atan(x.v.z()/x.v.x());
    double beta = std::asin(x.v.y()/vel_mod);
    
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);

    const double omp = x.omega.x();
    const double omq = x.omega.y();
    const double omr = x.omega.z();

    double epsilon = p.epsilon_alpha * (alpha - p.alpha_0);
    double alpha_t = alpha - epsilon + u.delta_E + p.CL_q_V * omq * p.l_t / vel_mod; 

    double CL_wb = p.CL_alpha * (alpha - p.alpha_0);
    double CL_t = p.CL_alpha_t * alpha_t * p.S_t / p.S ;
    double CL = CL_t + CL_wb;

    double CD = p.CD_0 + p.CD_2 * square(p.CD_alpha * alpha + p.CD_1);

    double CY = p.CY_beta * beta + p.CY_delta_R * u.delta_R;

    double Cl = p.Cl_beta * beta + p.Cl_p * omr * p.ch / vel_mod + p.Cl_delta_A * u.delta_A + p.Cl_delta_R * u.delta_R; 
    double Cm = p.Cm_0 + (p.Cm_alpha * (alpha - epsilon) + p.Cm_q * p.l_t * omq / vel_mod + p.Cm_delta_E * u.delta_E ) * p.S_t * p.l_t /(p.S * p.ch);
    double Cn = p.Cn_beta * beta + p.Cn_alpha_beta * alpha * beta + p.Cn_p * p.ch * omp / vel_mod + p.Cn_r * p.ch * omr / vel_mod + p.Cn_delta_R * u.delta_R;

    double qh = p.air_density * vel_mod * vel_mod / 2;

    double D = qh * p.S * CD;
    double L = qh * p.S * CL;
    double Y = qh * p.S * CY;
 
    Vector3d F_a = Vector3d::Zero();
    F_a.x() = L * sin_a - D * cos_a * cos_b - Y * cos_a * sin_b;
    F_a.y() = -D * sin_b + Y * cos_b;
    F_a.z() = -L * cos_a - D * sin_a * cos_b - Y * sin_a * sin_b;

    Vector3d F_e = Vector3d(2 * x.delta_t * p.mass * g, 0, 0);

    Vector3d F_g = R_i2b * Vector3d(0, 0, p.mass * g);

    Vector3d F_total = F_a + F_e + F_g;

    double LH = qh * p.S * p.ch * Cl;
    double MH = qh * p.S * p.ch * Cm;
    double NH = qh * p.S * p.ch * Cn;

    Vector3d M_a = Vector3d(LH, MH, NH) + F_a.cross(p.p_cg - p.p_ac);
    Vector3d M_e = (p.p_cg - p.p_t);

    x_dot.p =  R_b2i * x.v + p.wind_vel; // !!!!!!!!!!!11
    x_dot.v = F_total / p.mass - x.omega.cross( x.v.transpose() ); // !!!!!!!!!!!11
    x_dot.phi= inverseKinematicJacobian(x.phi) * x.omega; // !!!!!!!!!!!11
    x_dot.omega = p.moment_inertia.inverse() * (M_e + M_a - x.omega.cross(p.moment_inertia * x.omega)); // !!!!!!!!!!!11
    x_dot.delta_t = (u.eta_T - x.delta_t)/p.tau_t; // !!!!!!!!!!!11
}