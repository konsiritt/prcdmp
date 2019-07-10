//
// Created by Elie Aljalbout on 17.11.18.
//

#include "DMP/DMP.hpp"


DMP::DMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
         std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern)
  :cs(dt), nDMPs(nDMPs), nBFs(nBFs), dt(dt), y0(y0), goal(goal), couplingTerm(nDMPs,0.0), endThreshold(0.01), trajFinished(false), doSimpleRollout(false)
{
    if (w.size()>=nDMPs) // ugly, but somehow w.size is 8 instead of 7, must be sth with reading file
    {
        if (w[0].size()==nBFs)
        {
            this->w = w;
        }
    }

    this->gainA = gainA.size()? gainA : std::vector<double>(this->nDMPs, 25.0);
    if(gainB.size())
    {
        this->gainB = gainB;
    }
    else
    {
        for(int i=0;i<this->nDMPs;i++)
        {
            this->gainB.push_back(this->gainA[i]/4.0);
        }
    }

    this->cs = CanonicalSystem(dt,pattern);
    this->timesteps = (int) this->cs.getRunTime()/this->dt;
    resettState();
}

DMP::DMP(int nDMPs, double dt, std::vector<double> &y0, std::vector<double> &goal, std::vector<double> &gainA,
         std::vector<double> &gainB, std::string pattern)
 :cs(dt), nDMPs(nDMPs), nBFs(0), dt(dt), y0(y0), goal(goal), couplingTerm(nDMPs,0.0), endThreshold(0.01), trajFinished(false), doSimpleRollout(false)
{
    std::vector<std::vector<double>> wTemp(nDMPs,std::vector<double>(0, 0.0));
    w = wTemp;

    this->gainA = gainA.size()? gainA : std::vector<double>(this->nDMPs, 25.0);
    if(gainB.size())
    {
        this->gainB = gainB;
    }
    else
    {
        for(int i=0;i<this->nDMPs;i++)
        {
            this->gainB.push_back(this->gainA[i]/4.0);
        }
    }

    this->cs = CanonicalSystem(dt,pattern);
    this->timesteps = (int) this->cs.getRunTime()/this->dt;
    resettState();
}

void DMP::resettState()
{
    y   = y0;
    dy  = std::vector<double> (nDMPs, 0.0);
    ddy = std::vector<double> (nDMPs, 0.0);
    cs.resettState();
    trajFinished = false;
}

void DMP::checkOffset()
{
    for(int i=0; i < nDMPs; i++)
    {
        if(y0[i]==goal[i])
        {
            goal[i]+= 1e-3;
        }
    }
}

void DMP::rollout(std::vector<std::vector<double>> &yTrack, std::vector<std::vector<double>> &dyTrack, std::vector<std::vector<double>> &ddyTrack,//outputs
                  std::vector<double> externalForce, double tau, int timeSteps, double error )
{

    if (w.size()<nDMPs) //TODO: this is temporary, should be w.size() == nDMPs
    {
        if (nBFs == 0) {
            doSimpleRollout = true;
        }
        else {
            std::cerr<< "ATTENTION: Weights Matrix not initialized, not of the correct size";
            return;
        }
    }
    resettState();

    if (timeSteps==-1)
    {
        timeSteps = this->timesteps/tau *1.1; //TODO: add a stopping criterion?! 1.1: this is to assure that there are enough steps rolled out
    }

    for (int k=0;k<7;k++)
    {
        std::cout<<"Dimension["<<k<<"]: "<<" goal: "<<goal[k]<<" y0: "<<y0[k]<<std::endl;
    }

    int maxDur =0;
    for (int t=0; t<timeSteps; t++)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        if (doSimpleRollout)
        {
            simpleStep(externalForce, tau);
        }
        else
        {
            step(externalForce, tau, error);
        }
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
    //std::cout<<maxDur<< "maximum duration is <----"<<std::endl;
}

std::vector<double> DMP::step( std::vector<double> &externalForce, double tau, double error)
{
    if (w.size()==0)
    {
        throw "ATTENTION: Weights Matrix not initialized";
    }

    double errorCoupling = 1.0/(1.0+error);

    double x    = cs.step(tau, errorCoupling);
    if (x < endThreshold) {trajFinished = true;}
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

        this->ddy[i] = tau*tau*(gainA[i]* (gainB[i]*(goal[i]-this->y[i] -(goal[i] - y0[i])*x +f ) -this->dy[i]/tau) )
                                + couplingTerm[i]; //2009

        if(externalForce.size()==nDMPs)
        {
            this->ddy[i] += externalForce[i];
        }
        this->dy[i] += this->ddy[i]*dt*errorCoupling;
        this->y[i]  += this->dy[i]*dt*errorCoupling;
    }
    return this->dy;
}

std::vector<double> DMP::simpleStep( std::vector<double> &externalForce, const double &tau)
{
//    std::cout<<"DMP simple Step: cs";
    double x    = cs.step(tau, 1.0);

    if (x < endThreshold) {trajFinished = true;}

//    std::cout<<"; it";
    for (int i=0; i<nDMPs; i++)
    {
//        std::cout<<"i: "<<i;
	//specifically for a constant function: goal and initial value are the same, to prevent small number errors
        if (abs(this->y[i] - goal[i]) < 1e-6)
        {
            this->ddy[i] = 0.0;
            this->dy[i] = 0.0;
        }
        else
        {
            this->ddy[i] = tau*tau*(gainA[i]* (gainB[i]*(goal[i]-this->y[i] -(goal[i] - y0[i])*x ) -this->dy[i]/tau));

            this->dy[i] += this->ddy[i]*dt;
        }

        this->y[i]  += this->dy[i]*dt;
    }
//    std::cout<<"; return"<<std::endl;
    return this->dy;
}

int DMP::getTimesteps()
{
    return timesteps;
}

std::vector<double> DMP::getY()
{
    return y;
}

std::vector<double> DMP::getDY()
{
    return dy;
}

bool DMP::getTrajFinished()
{
  return trajFinished;
}

void DMP::setCouplingTerm(std::vector<double> &couplTerm)
{
    if (couplTerm.size()==nDMPs) {
        couplingTerm = couplTerm;
    }
    //std::cout<<"setCouplingTerm: c[0]="<<couplingTerm[0]<<"c[1]="<<couplingTerm[1]<<"c[6]="<<couplingTerm[6]<<std::endl;
}

