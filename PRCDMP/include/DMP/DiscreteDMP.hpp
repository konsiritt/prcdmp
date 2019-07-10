//
// Created by Elie Aljalbout on 18.11.18.
//

#ifndef PROJECT_LINEARDMP_HPP
#define PROJECT_LINEARDMP_HPP

#include <math.h>
#include "DMP.hpp"
#include "UTILS/trajectoryUtils.h"
#include <iostream>

class DiscreteDMP : public DMP
{
public:
    /**
     * @brief DiscreteDMP generates a discrete rather than periodic dmp
     * @param nDMPs amount of dmp DOF
     * @param nBFs number of basis functions
     * @param dt timestep size
     * @param y0 intial position
     * @param goal final position
     * @param w weights of the basis functions
     * @param gainA alpha gain
     * @param gainB beta gain
     * @param pattern
     */
    DiscreteDMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
                std::vector<std::vector<double>> &w, std::vector<double> &gainA,
                std::vector<double> &gainB, std::string pattern ="discrete");

    DiscreteDMP(int nDMPs, double dt, std::vector<double> &y0, std::vector<double> &goal,
                std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern ="discrete");

    DiscreteDMP(){std::cerr<<"created an empty instance of DiscreteDMP"<<std::endl;};

    ~DiscreteDMP(){};

    void setInitialPosition (std::vector<double> &y_0);

    void setFinalPosition (std::vector<double> &y_end);

    void setEndThreshold (double thrsh);

//    std::vector<double> step( std::vector<double> &externalForce, double tau=1.0, double error=0.0);

//    std::vector<double> simpleStep( std::vector<double> &externalForce, double tau=1.0, double error=0.0);
    void writeTrajToText(const std::vector<std::vector<double>> &traj, std::string file_name);


protected:
    /// generates the Gaussian basis function
    void genPSI(const double &x, std::vector<double> &psi);
    /// generates (almost equally distributed across x) centers for gaussians
    void genCenters();

private:

};


#endif //PROJECT_LINEARDMP_HPP
