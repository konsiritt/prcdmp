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

    /**
     * @brief DMP: gives a dynamic movement primitive, describing a trajectory
     * @param nDMPs: dimensions of the dmp, degrees of freedom of described trajectory
     * @param nBFs: amount of basis functions for the dmp
     * @param dt: size of time step
     * @param y0: vector of initial positions
     * @param goal: vector of goal positions that are achieved by attractor dynamics
     * @param w: matrix of weighting terms for basis functions that force the trajectory to its shape
     * @param gainA: gain parameter for the attractor dynamics
     * @param gainB: gain parameter for the attractor dynamics
     * @param pattern: discrete or cyclic
     */
    DMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
            std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern="discrete");

    /**
     * @brief DMP: same as before, no weighting terms -> leads to simple attractor dynamics (critically damped)
     */
    DMP(int nDMPs, double dt, std::vector<double> &y0, std::vector<double> &goal, std::vector<double> &gainA,
        std::vector<double> &gainB, std::string pattern="discrete");

    DMP(){std::cerr<<"created an empty instance of DMP"<<std::endl;};
   
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
     * @return the dmp velocity vector
     */
    virtual std::vector<double> step( std::vector<double> &externalForce, double tau=1.0, double error=0.0);

    /**
     * @brief simpleStep steps only the attractor dynamics of the system without forcing term
     * @param externalForce
     * @param tau
     * @param error
     * @return the dmp velocity vector
     */
    virtual std::vector<double> simpleStep(std::vector<double> &externalForce, const double &tau=1.0);

    /**
     * @brief resettState resets all parameters to allow a restart of the dmp
     */
    virtual void resettState();

    /**
     * @brief getTimesteps
     * @return amount of timesteps involved over the course of the dmp
     */
    int getTimesteps();

    /**
     * @brief getDY
     * @return the dmp velocities/ change in position
     */
    std::vector<double> getY();
    /**
     * @brief getDY
     * @return the dmp velocities/ change in position
     */
    std::vector<double> getDY();

    /**
     * @brief getTrajFinished returns boolean that determines end of the trajectory on the basis of
     * the canonical system reaching below a certain threshold -> the system dynamics do not change anymore
     * @return
     */
    bool getTrajFinished();

    /**
     * @brief setCouplingTerm
     * @param couplTerm: adapts the coupling term to be used in the dmp
     */
    void setCouplingTerm(std::vector<double> &couplTerm);

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
    /// coupling term per dof
    std::vector<double> couplingTerm;
    /// Number of DMP DOF
    int nDMPs;
    /// Number of basis functions
    int nBFs;
    /// threshold to determine end of trajectory, compared to the canonical system state
    double endThreshold;
    /// flag to determine end of trajectory
    bool trajFinished;
    /// amount of timesteps for the trajectory
    int timesteps;
    /// timestep size
    double dt;
    /// weights of the forcing term
    std::vector<std::vector<double>> w;

    CanonicalSystem cs;

    /// generates the Gaussian basis function
    virtual void genPSI(const double &x, std::vector<double> &psi)=0;

    /// generates (almost equally distributed across x) centers for gaussians
    virtual void genCenters()=0;

    /// checks how far goal and initial state are apart to avoid problems
    virtual void checkOffset();

private:
    bool doSimpleRollout;
 };

#endif //PROJECT_DMP_H
