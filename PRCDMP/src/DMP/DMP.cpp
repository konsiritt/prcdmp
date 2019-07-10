//
// Created by Elie Aljalbout on 17.11.18.
//

#include "DMP/DMP.hpp"


DMP::DMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
         std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern)
:cs(dt), nDMPs(nDMPs), nBFs(nBFs), dt(dt), y0(y0), goal(goal)
{
    if (w.size())
    {
        this->w = w;
    }

    this->gainA = gainA.size()? gainA : std::vector<double>(this->nDMPs, 25);
    if(gainB.size())
    {
        this->gainB = gainB;
    }
    else
    {
        for(int i=0;i<this->nDMPs;i++)
        {
            this->gainB.push_back(this->gainA[i]/4);
        }
    }
    this->cs = CanonicalSystem(dt,pattern);
    this->timesteps = (int) this->cs.getRunTime()/this->dt;
    resettState();
}

void DMP::resettState()
{
    y   = y0;
    dy  = std::vector<double> (nDMPs, 0);
    ddy = std::vector<double> (nDMPs, 0);
    cs.resettState();
}

void DMP::checkOffset()
{
    for(int i=0; i < nDMPs; i++)
    {
        if(y0[i]==goal[i])
        {
            goal[i]+= 1e-4;
        }
    }
}

std::vector<double> DMP::step( std::vector<double> &externalForce, double tau, double error)
{
    if (w.size()==0)
    {
        throw "ATTENTION: Weights Matrix not initialized";
    }

    double errorCoupling = 1.0/(1.0+error);

    double x    = cs.step(tau, errorCoupling);
    std::vector<double> psi;
    genPSI(x, psi);

    double f;
    int maxDur=0;
    for (int i=0; i<nDMPs; i++)
    {        
        double in, acc;
        in = inner_product(&psi[0], &w[i][0], psi.size());
        acc = accumulate(&psi[0], psi.size() );
        f = x * in / acc;

        this->ddy[i] = (tau*tau*gainA[i]* (gainB[i]*(goal[i]-this->y[i] -(goal[i] - y0[i])*x +f ) -this->dy[i]/tau  )); //2009

        if(externalForce.size()==nDMPs)
        {
            this->ddy[i] += externalForce[i];
        }
        this->dy[i] += this->ddy[i]*dt*errorCoupling;
        this->y[i]  += this->dy[i]*dt*errorCoupling;
    }
    return this->dy;
}

void DMP::rollout(std::vector<std::vector<double>> &yTrack, std::vector<std::vector<double>> &dyTrack, std::vector<std::vector<double>> &ddyTrack,//outputs
                  std::vector<double> externalForce, double tau, int timeSteps, double error )
{

    if (w.size()==0)
    {
        std::cerr<< "ATTENTION: Weights Matrix not initialized";
        return;
    }
    resettState();

    if (timeSteps==-1)
    {
        timeSteps = this->timesteps/tau;
    }

    for (int k=0;k<7;k++)
    {
        std::cout<<"Dimension["<<k<<"]: "<<" goal: "<<goal[k]<<" y0: "<<y0[k]<<std::endl;
    }

    int maxDur =0;
    for (int t=0; t<timeSteps; t++)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        step(externalForce, tau, error);
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        if(duration>maxDur)
        {
            maxDur = (int) duration;
        }
        yTrack.push_back(y);
        dyTrack.push_back(dy);
        ddyTrack.push_back(ddy);
    }
    std::cout<<maxDur<< "maximum duration is <----"<<std::endl;
}

int DMP::getTimesteps()
{
    return timesteps;
}


std::vector<double> DMP::getDY()
{
    return dy;
}
