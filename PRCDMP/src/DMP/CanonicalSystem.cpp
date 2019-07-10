//
// Created by Elie Aljalbout on 17.11.18.
//

#include "DMP/CanonicalSystem.hpp"

CanonicalSystem::CanonicalSystem(double dt, std::string pattern, double gainA)
    : gainA(gainA), pattern(pattern), dt(dt)
{
    if(this->pattern == "discrete")
    {
        this->runTime = 1.0;
    }
    else if (this->pattern == "rhythmic")
    {
        this->runTime = 2*M_PI;
    }

    this->timesteps = this->runTime/this->dt;

    resettState();
}

void CanonicalSystem::resettState()
{
    x = 1.0;
}

void CanonicalSystem::rollout(double tau, std::vector<double> & xTrack)
{
    int timesteps = (int) this->timesteps/tau;

    for (int i =0;i<timesteps;i++)
    {
        xTrack.push_back(this->x);
        step(tau);
    }
}

void CanonicalSystem::rollout( std::vector<double> & xTrack)
{
    int timesteps = this->timesteps;

    for (int i =0;i<timesteps;i++)
    {
        xTrack.push_back(this->x);
        step();
    }
}


double CanonicalSystem::step(double tau, double couplingError)
{
    if (pattern == "discrete")
    {
        this->x += (-gainA*x*couplingError)*tau*dt;
    }
    else if (this->pattern == "rhythmic")
    {
        this->x += (couplingError*tau) * dt;
    }
    return this->x;
}

double CanonicalSystem::getRunTime()
{
    return this->runTime;
}

double CanonicalSystem::getGainA()
{
    return gainA;
}
