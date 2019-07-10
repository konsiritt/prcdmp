//
// Created by Elie Aljalbout on 17.11.18.
//

#ifndef PROJECT_DMP_H
#define PROJECT_DMP_H

#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>
#include "CanonicalSystem.hpp"
#include "UTILS/common.h"


using namespace std::chrono;
/**
 * Implementation of Dynamic Motor Primitives,
    as described "Learning and generalization of motor skills by learning from
    demonstration" by Pastor et. al. (2009)
 */
class DMP
{

public:

    DMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
            std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern="discrete");

    virtual ~DMP(){};

    /**
     * @brief rollout generates a dmp trajectory for the whole
     * @param [output] yTrack vector of vectors containing the trajectory [timestep][dimension]
     * @param [output] dyTrack vector of vectors containing the velocity of the trajectory
     * @param [output] ddyTrack vector of vectors containing the acceleration of the trajectory
     * @param externalForce
     * @param tau time scaling factor (<1 -> slowed down)
     * @param timeSteps
     * @param error
     */
    virtual void rollout(std::vector<std::vector<double>> &yTrack,
                         std::vector<std::vector<double>> &dyTrack,
                         std::vector<std::vector<double>> &ddyTrack, //outputs
                         std::vector<double> externalForce,
                         double tau=1.0, int timeSteps=-1, double error=0.0 ); //inputs

    /**
     * @brief step propagates the dynamical system of the dmp for one timestep
     * @param externalForce
     * @param tau time scaling factor (<1 -> slowed down)
     * @param error
     * @return
     */
    virtual std::vector<double> step( std::vector<double> &externalForce, double tau=1.0, double error=0.0);

    virtual void resettState();

    int getTimesteps();

    std::vector<double> getDY();

protected:

    /// gaussian basis function center
    std::vector<double> centers;
    /// gaussian basis function variance
    std::vector<double> vars;
    /// vector (in all DOF dimensions) of the dmp position
    std::vector<double> y;
    /// dmp velocity
    std::vector<double> dy;
    /// dmp acceleration
    std::vector<double> ddy;
    /// initial positions
    std::vector<double> y0;
    /// final/goal positions
    std::vector<double> goal;
    /// gain alpha
    std::vector<double> gainA;
    /// gain beta
    std::vector<double> gainB;
    /// Number of DMP DOF
    int nDMPs;
    /// Number of basis functions
    int nBFs;

    CanonicalSystem cs;

    /// generates the Gaussian basis function
    virtual void genPSI(const double &x, std::vector<double> &psi)=0;

    /// generates (almost equally distributed across x) centers for gaussians
    virtual void genCenters()=0;

    /// checks how far goal and initial state are apart to avoid problems
    virtual void checkOffset();

private:
    /// amount of timesteps for the trajectory
    int timesteps;
    /// timestep size
    double dt;
    /// weights of the forcing term
    std::vector<std::vector<double>> w;
 };

#endif //PROJECT_DMP_H
