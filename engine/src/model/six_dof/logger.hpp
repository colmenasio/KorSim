#include <fstream>
#include <iomanip>
#include <iostream>
#include <Eigen/Dense>
#include "model.hpp"

namespace korsim::six_dof
{
    class Logger
    {
    public:
        Logger(const std::string &filename) : file_(filename)
        {
        }

        ~Logger()
        {
            if (file_.is_open())
            {
                file_.close();
            }
        }

        Logger(Logger& other) = delete;
        Logger(Logger&& other) = delete;
        Logger& operator=(Logger& other) = delete;
        Logger& operator=(Logger&& other) = delete;

        void log(const std::string &str)
        {
            if (!file_.is_open())
                return;
            file_ << str;
        }

        void log(const State &state)
        {
            if (!file_.is_open())
                return;

            file_ << "p: " << state.p.transpose() << "\n";
            file_ << "v: " << state.v.transpose() << "\n";
            file_ << "phi: " << state.phi.transpose() << "\n";
            file_ << "omega: " << state.omega.transpose() << "\n";
            file_ << "delta_t: " << state.delta_t << "\n";
        }

        void log(const Model &obj)
        {
            if (!file_.is_open())
                return;

            auto state = obj.getDynamics();

            log("X_DOT\n");
            log(state);

            file_ << "F_e: " << obj.getEngineForce().transpose() << "\n";
            file_ << "F_a: " << obj.getAerodynamicForce().transpose() << "\n";
            file_ << "F_g: " << obj.getGravityForce().transpose() << "\n";

            file_ << "M_e: " << obj.getEngineMoment().transpose() << "\n";
            file_ << "M_a: " << obj.getAerodynamicMoment().transpose() << "\n";

            // Aerodynamic coefficients and angles
            file_ << obj.getAngleOfAttack() << ","
                  << obj.getAngleOfAttackTail() << ","
                  << obj.getSideslipAngle() << "\n";

            file_ << obj.getLiftCoeff() << ","
                  << obj.getDragCoeff() << ","
                  << obj.getSideforceCoeff() << "\n";

            file_ << obj.getRollingMomentCoeff() << ","
                  << obj.getPitchingMomentCoeff() << ","
                  << obj.getYawingMomentCoeff() << "\n";
        }

    private:
        std::ofstream file_;
    };
}