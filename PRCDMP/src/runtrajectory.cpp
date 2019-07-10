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


int main(int argc, char** argv)
{
    int error = parseArgs(argc,argv); //check arguments and modify them to fit rest of the script (remove / from beginning and end of dataset name)
    if (error!=-1) return error;

    std::string datasetPath = argv[1];

    Config config(datasetPath);

    //fill data from json to variables
    int dofs = config.getDmpJson()["dofs"].asInt();
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    std::string robotIp = config.getDataJson()["robot_ip"].asString();
    std::cout<<"Robot ip : "<<robotIp<<std::endl;
    std::array<double,7> q0;
    moveJsonArrayToVec(config.getDmpJson()["q0"], q0);

    std::cout<<"current episode: "<<episodeNr<<std::endl;
    config.fillTrajectoryPath(episodeNr);

    Interface roboInter;

    std::vector<std::vector<double>> yTrack;
    std::vector<std::vector<double>> dyTrack;
    std::vector<std::vector<double>> ddyTrack;

    roboInter.runTrajectory(robotIp, q0, config.getReplayTrajectoryPath(), dofs);
    return 0;
}

