//  main.cpp
//  PRCDMP
//
//  Created by Elie Aljalbout on 04.10.18.
//  Copyright © 2018 Elie Aljalbout. All rights reserved.

#include <iostream>
#include <fstream>
#include <string>
#include "DMP/DMP.hpp"
#include "DMP/DiscreteDMP.hpp"
#include "UTILS/trajectoryUtils.h"
#include "UTILS/Config.h"
#include <vector>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
#include "interface.h"

using namespace std;

int parseArgs(int argc, char** argv)
{
    if(argc<2)
    {
        std::cerr<<"ERROR: Please provide the path to the dataset repository starting from prcdmp/data";
        return 669;
    }

    std::string datasetPath = argv[1];

    if(datasetPath.at(0)=='/') // Make sure only the name of the dataset not a bounded path is given setx instead of /setx/
    {
        datasetPath = datasetPath.substr(1);
    }

    if(datasetPath.at(datasetPath.length()-1) == '/')
    {
        datasetPath = datasetPath.substr(0,datasetPath.length()-1);
    }
    return -1;
}

// @todo: init function...
int main(int argc, char** argv)
{
    int error = parseArgs(argc,argv); //check arguments and modify them to fit rest of the script (remove / from beginning and end of dataset name)
    if (error!=-1) return error;

    std::string datasetPath = argv[1];

    // handles config file access
    Config config(datasetPath);

    //fill data from json to variables
    int dofs = config.getDmpJson()["dofs"].asInt();
    int nBFs = config.getDmpJson()["n_basis"].asInt();
    double dt = config.getDmpJson()["dt"].asDouble();
    double timeSpan = config.getDmpJson()["timespan"].asDouble();
    double tau = 1.0/timeSpan;

    // initialize arrays from config file
    std::array<double,7> q0;
    std::array<double,7> goal;
    std::vector<double> gainA, gainB;
    moveJsonArrayToVec(config.getDmpJson()["q0"], q0);
    moveJsonArrayToVec(config.getDmpJson()["goal"], goal);
    moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
    moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);

    //fill data from json to variables
    std::string robotIp = config.getDataJson()["robot_ip"].asString();
    std::cout<<"Robot ip : "<<robotIp<<std::endl;
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    config.fillTrajectoryPath(episodeNr);

    Interface roboInter;
    std::vector<double> externalForce;
    std::vector<std::vector<double>> w ;
    if (episodeNr ==0) {
        UTILS::loadWeights(config.getInitialWPath(),w);
    }
    else {
        UTILS::loadWeights(config.getwPath(),w);
    }

    std::vector<std::vector<double>> wTemp(w.size(), std::vector<double>(w[0].size(),0.0));

    // convert arrays to vectors
    std::vector<double> y0v(q0.begin(), q0.end());
    std::vector<double> goalv(goal.begin(), goal.end());

    DiscreteDMP dmp(dofs, nBFs, dt, y0v, goalv, wTemp, gainA, gainB);

    std::vector<std::vector<double>> yTrack;
    std::vector<std::vector<double>> dyTrack;
    std::vector<std::vector<double>> ddyTrack;

    //@todo: Add if debug rollout else runTraj
    dmp.rollout(yTrack, dyTrack, ddyTrack, externalForce, tau, -1, 0 );
    std::cout<<"writing reconstructed trajectory to file: "<<config.getReplayTrajectoryPath()<<std::endl;
    UTILS::writeTrajToText(dyTrack,config.getReplayTrajectoryPath());
    UTILS::writeTrajToText(yTrack,config.getReplayTrajectoryPathQ());
    //roboInter.runDMPTrajectory(dmp, robotIp, externalForce, q0, -1, tau);
    return 0;
}

