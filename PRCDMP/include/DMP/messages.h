//
// Created by Elie Aljalbout on 06.10.18.
//

#include <vector>

#ifndef RLF_MESSAGES_H
#define RLF_MESSAGES_H

struct DMPPoint{

    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};

struct DMPTraj{

    std::vector<DMPPoint> points;
    std::vector<double> times;
};

struct DMPData{

    double k_gain;
    double d_gain;
    std::vector<double> weights;
    std::vector<double > f_domain;
    std::vector<double > f_targets;
};

//Contains DMPdata that is class-specific
struct OptionalDMPData
{
    int num_bases; // For radial DMPs
    double base_width; // for radial DMPs
    double alpha; // for radial DMPs
    int order; // for Fourier DMPS
};

#endif //RLF_MESSAGES_H
