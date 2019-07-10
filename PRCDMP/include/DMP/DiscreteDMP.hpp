//
// Created by Elie Aljalbout on 18.11.18.
//

#ifndef PROJECT_LINEARDMP_HPP
#define PROJECT_LINEARDMP_HPP

#include <math.h>
#include "DMP.hpp"
#include "UTILS/trajectoryUtils.h"

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

    ~DiscreteDMP(){};

protected:
    /// generates the Gaussian basis function
    void genPSI(const double &x, std::vector<double> &psi);
    /// generates (almost equally distributed across x) centers for gaussians
    void genCenters();

private:

};


#endif //PROJECT_LINEARDMP_HPP
