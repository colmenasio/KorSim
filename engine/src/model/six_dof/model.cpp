#include "model.hpp"

#include "iostream"

using namespace Eigen;
using namespace korsim::util;

inline double square(double x)
{
    return x * x;
}

void korsim::six_dof::Model::calculateDynamics(const State &x, const Control &u, const Parameters &p)
{
    const double g = 9.8;

    Matrix3d R_b2i = rotationFromEuler(x.phi);
    Matrix3d R_i2b = R_b2i.transpose();

    double vel_mod = x.v.norm();
    vel_mod += 1e-4; // ipsilo omega ipslion

    // igual conviene interpolar angulos pa que evitar discontinuidades dfk
    beta = std::asin(x.v.y() / vel_mod);
    if (std::abs(x.v.x()) < 1e-5 && std::abs(x.v.z()) < 1e-5)
    {
        alpha = 0;
    }
    else
    {
        alpha = std::atan2(x.v.z(), x.v.x());
    }

    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);

    const double omp = x.omega.x();
    const double omq = x.omega.y();
    const double omr = x.omega.z();

    double epsilon = p.epsilon_alpha * (alpha - p.alpha_0);

    alpha_t = alpha - epsilon + u.delta_E + p.CL_q_V * omq * p.l_t / vel_mod;  
    
    std::cout << "meow :3: " << std::endl;
    std::cout << "vel: " << x.v.transpose() << std::endl;
    std::cout << "aoa: " << alpha << std::endl;
    std::cout << "aoa tail: " << alpha_t << std::endl;
    std::cout << "epsilon: " << epsilon << std::endl;

    double CL_wb = p.CL_alpha * (alpha - p.alpha_0);
    double CL_t = p.CL_alpha_t * alpha_t * p.S_t / p.S;
    CL = CL_t + CL_wb;
    CD = p.CD_0 + p.CD_2 * square(p.CD_alpha * alpha + p.CD_1);
    CY = p.CY_beta * beta + p.CY_delta_R * u.delta_R;

    std::cout << "CL_wb: " << CL_wb << std::endl;
    std::cout << "CL_t: " << CL_t  << std::endl;

    Cl = p.Cl_beta * beta + p.Cl_p * omr * p.ch / vel_mod + p.Cl_delta_A * u.delta_A + p.Cl_delta_R * u.delta_R;
    Cm = p.Cm_0 + (p.Cm_alpha * (alpha - epsilon) + p.Cm_q * p.l_t * omq / vel_mod + p.Cm_delta_E * u.delta_E) * p.S_t * p.l_t / (p.S * p.ch);
    Cn = p.Cn_beta * beta + p.Cn_alpha_beta * alpha * beta + p.Cn_p * p.ch * omp / vel_mod + p.Cn_r * p.ch * omr / vel_mod + p.Cn_delta_R * u.delta_R;

    double qh = p.air_density * vel_mod * vel_mod / 2;
    std::cout << "qh: " << qh << std::endl;

    double D = qh * p.S * CD;
    double L = qh * p.S * CL;
    double Y = qh * p.S * CY;
    std::cout << "L: " << L << std::endl;

    F_L = L * Vector3d(sin_a, 0, -cos_a);
    std::cout << "FL: " << F_L.transpose() << std::endl;
    F_D = D * Vector3d(-cos_a * cos_b, -sin_b, - sin_a * cos_b);
    F_Y = Y * Vector3d(-cos_a * sin_b, cos_b, - sin_a * sin_b);

    F_a = F_L + F_D + F_Y;

    F_e = Vector3d(2 * x.delta_t * p.mass * g, 0, 0);

    F_g = R_i2b * Vector3d(0, 0, p.mass * g);

    Vector3d F_total = F_a + F_e + F_g;

    double LH = qh * p.S * p.ch * Cl;
    double MH = qh * p.S * p.ch * Cm;
    double NH = qh * p.S * p.ch * Cn;

    M_a = Vector3d(LH, MH, NH) + F_a.cross(p.p_cg - p.p_ac);
    M_e = (p.p_cg - p.p_t).cross(F_e);

    x_dot.p = R_b2i * x.v + p.wind_vel;                                                                 // !!!!!!!!!!!11
    x_dot.v = F_total / p.mass - x.omega.cross(x.v);                                                    // !!!!!!!!!!!11
    x_dot.phi = inverseKinematicJacobian(x.phi) * x.omega;                                              // !!!!!!!!!!!11
    x_dot.omega = p.moment_inertia.inverse() * (M_e + M_a - x.omega.cross(p.moment_inertia * x.omega)); // !!!!!!!!!!!11
    x_dot.delta_t = (u.eta_T - x.delta_t) / p.tau_t;                                                    // !!!!!!!!!!!11
}