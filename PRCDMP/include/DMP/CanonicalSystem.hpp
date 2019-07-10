                 //
// Created by Elie Aljalbout on 17.11.18.
//

#ifndef PROJECT_CANONICALSYSTEM_HPP
#define PROJECT_CANONICALSYSTEM_HPP


#include <string>
#include <math.h>
#include <vector>

/**
 * @brief The CanonicalSystem class covers the phase description of the dmps
 */
class CanonicalSystem
{

public:

    /// gainA is chosen such, that x(t=1.0)=0.01
    CanonicalSystem(double dt, std::string pattern="discrete", double gainA=4.60517);//1.0);

    void resettState();

    void rollout(std::vector<double> & xTrack);

    void rollout(double tau, std::vector<double> & xTrack);

    double step(double tau=1.0, double couplingError = 1.0);

    double getRunTime();

    double getGainA();

private:

    double x;
    double gainA;
    std::string pattern;
    double dt;
    int timesteps;
    double runTime;
};


#endif //PROJECT_CANONICALSYSTEM_HPP
