#pragma once

#include <Eigen/Dense>

namespace korsim::six_dof
{

    struct Parameters
    {
        /* data */
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
        Parameters p;

    public:
        Model(const Parameters &parameters) : p(parameters) {}

        // Calculates the dynamics and stores all results internally. Said results can be obtained with their respective getters
        void calculateDynamics(const State& x, const Control& u);

    private:
    };

}