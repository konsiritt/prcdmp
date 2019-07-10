//
// Created by Elie Aljalbout on 18.11.18.
//

#include "DMP/DiscreteDMP.hpp"


DiscreteDMP::DiscreteDMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
                         std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern)
            : DMP(nDMPs, nBFs, dt, y0, goal, w, gainA, gainB, pattern)
{
    genCenters();

    //Set variance of gaussian basis functions
    double var = nBFs*sqrt(nBFs) / cs.getGainA();
    for (int b=0; b<nBFs; b++)
    {
        vars.push_back(var/centers[b]);
    }

    checkOffset();
}

void DiscreteDMP::genCenters()
{
    std::vector<double> desCenters = UTILS::linspace<double>(0,  cs.getRunTime(), nBFs);

    for (int i=0; i<desCenters.size(); i++)
    {
        centers.push_back(exp(-cs.getGainA()*desCenters[i]));
    }
}

void DiscreteDMP::genPSI(const double &x, std::vector<double> &psi)
{
    psi.clear();
    for (int i=0; i<nBFs; i++)
    {
        psi.push_back(exp( -vars[i] * pow((x-centers[i]),2)));
    }

}
