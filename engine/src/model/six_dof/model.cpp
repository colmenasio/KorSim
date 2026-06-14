#include "model.hpp"

using namespace Eigen;
using namespace korsim::util;

#include <iostream>

inline double square(double x)
{
    return x * x;
}

void korsim::six_dof::Model::calculateDynamics(const State &x, const Control &u, const Parameters &p)
{
    const double g = 9.8;

    Matrix3d R_i2b = rotationFromEuler(x.phi);
    Matrix3d R_b2i = R_i2b.transpose();
    double vel_mod = x.v.norm();

    // igual conviene interpolar angulos pa que evitar discontinuidades dfk
    double alpha = 0;
    double beta = 0;
    if (x.v.x() > 1e5 && x.v.z() > 1e5 ){
        alpha = std::atan2(x.v.z(), x.v.x());
    }
    if (vel_mod > 1e5){
        beta = std::asin(x.v.y() / vel_mod);
    }

    std::cout << "=== DEBUG: Input State ===" << std::endl;
    std::cout << "vel_mod = " << vel_mod << std::endl;
    std::cout << "alpha = " << alpha << " rad (" << alpha * 180 / M_PI << " deg)" << std::endl;
    std::cout << "beta = " << beta << " rad (" << beta * 180 / M_PI << " deg)" << std::endl;
    std::cout << "x.v = [" << x.v.x() << ", " << x.v.y() << ", " << x.v.z() << "]" << std::endl;
    std::cout << "x.omega = [" << x.omega.x() << ", " << x.omega.y() << ", " << x.omega.z() << "]" << std::endl;

    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);

    std::cout << "sin_a = " << sin_a << ", cos_a = " << cos_a << std::endl;
    std::cout << "sin_b = " << sin_b << ", cos_b = " << cos_b << std::endl;

    const double omp = x.omega.x();
    const double omq = x.omega.y();
    const double omr = x.omega.z();

    std::cout << "omp = " << omp << ", omq = " << omq << ", omr = " << omr << std::endl;

    double epsilon = p.epsilon_alpha * (alpha - p.alpha_0);
    std::cout << "epsilon = " << epsilon << std::endl;

    double alpha_t = alpha - epsilon + u.delta_E + p.CL_q_V * omq * p.l_t / vel_mod;// ESTO EPLTA PERO TENGO DEMASIADO SUEÑO COMO PARA VER EL SENTIDO FISICO Y POR TANTO QUE VALOR ASINTOTICO DEBERIA TOMAR BUENAS NOCHES
    std::cout << "alpha_t = " << alpha_t << " rad (" << alpha_t * 180 / M_PI << " deg)" << std::endl;

    double CL_wb = p.CL_alpha * (alpha - p.alpha_0);
    double CL_t = p.CL_alpha_t * alpha_t * p.S_t / p.S;
    double CL = CL_t + CL_wb;

    std::cout << "CL_wb = " << CL_wb << ", CL_t = " << CL_t << ", CL = " << CL << std::endl;

    double CD = p.CD_0 + p.CD_2 * square(p.CD_alpha * alpha + p.CD_1);
    std::cout << "CD = " << CD << std::endl;

    double CY = p.CY_beta * beta + p.CY_delta_R * u.delta_R;
    std::cout << "CY = " << CY << std::endl;

    double Cl = p.Cl_beta * beta + p.Cl_p * omr * p.ch / vel_mod + p.Cl_delta_A * u.delta_A + p.Cl_delta_R * u.delta_R;
    double Cm = p.Cm_0 + (p.Cm_alpha * (alpha - epsilon) + p.Cm_q * p.l_t * omq / vel_mod + p.Cm_delta_E * u.delta_E) * p.S_t * p.l_t / (p.S * p.ch);
    double Cn = p.Cn_beta * beta + p.Cn_alpha_beta * alpha * beta + p.Cn_p * p.ch * omp / vel_mod + p.Cn_r * p.ch * omr / vel_mod + p.Cn_delta_R * u.delta_R;

    std::cout << "Cl = " << Cl << ", Cm = " << Cm << ", Cn = " << Cn << std::endl;

    double qh = p.air_density * vel_mod * vel_mod / 2;
    std::cout << "qh = " << qh << std::endl;

    double D = qh * p.S * CD;
    double L = qh * p.S * CL;
    double Y = qh * p.S * CY;

    std::cout << "D = " << D << ", L = " << L << ", Y = " << Y << std::endl;

    Vector3d F_a = Vector3d::Zero();
    F_a.x() = L * sin_a - D * cos_a * cos_b - Y * cos_a * sin_b;
    F_a.y() = -D * sin_b + Y * cos_b;
    F_a.z() = -L * cos_a - D * sin_a * cos_b - Y * sin_a * sin_b;

    std::cout << "F_a = [" << F_a.x() << ", " << F_a.y() << ", " << F_a.z() << "]" << std::endl;

    Vector3d F_e = Vector3d(2 * x.delta_t * p.mass * g, 0, 0);
    std::cout << "F_e = [" << F_e.x() << ", " << F_e.y() << ", " << F_e.z() << "]" << std::endl;
    std::cout << "x.delta_t = " << x.delta_t << ", p.mass = " << p.mass << std::endl;

    Vector3d F_g = R_i2b * Vector3d(0, 0, p.mass * g);
    std::cout << "F_g = [" << F_g.x() << ", " << F_g.y() << ", " << F_g.z() << "]" << std::endl;

    Vector3d F_total = F_a + F_e + F_g;
    std::cout << "F_total = [" << F_total.x() << ", " << F_total.y() << ", " << F_total.z() << "]" << std::endl;

    double LH = qh * p.S * p.ch * Cl;
    double MH = qh * p.S * p.ch * Cm;
    double NH = qh * p.S * p.ch * Cn;

    std::cout << "LH = " << LH << ", MH = " << MH << ", NH = " << NH << std::endl;

    Vector3d M_a = Vector3d(LH, MH, NH) + F_a.cross(p.p_cg - p.p_ac);
    std::cout << "M_a = [" << M_a.x() << ", " << M_a.y() << ", " << M_a.z() << "]" << std::endl;

    Vector3d M_e = (p.p_cg - p.p_t).cross(F_e);
    std::cout << "M_e = [" << M_e.x() << ", " << M_e.y() << ", " << M_e.z() << "]" << std::endl;

    std::cout << "=== End Debug ===" << std::endl;

    x_dot.p = R_b2i * x.v + p.wind_vel;                                                                 // !!!!!!!!!!!11
    x_dot.v = F_total / p.mass - x.omega.cross(x.v.transpose());                                        // !!!!!!!!!!!11
    x_dot.phi = inverseKinematicJacobian(x.phi) * x.omega;                                              // !!!!!!!!!!!11
    x_dot.omega = p.moment_inertia.inverse() * (M_e + M_a - x.omega.cross(p.moment_inertia * x.omega)); // !!!!!!!!!!!11
    x_dot.delta_t = (u.eta_T - x.delta_t) / p.tau_t;                                                    // !!!!!!!!!!!11
}