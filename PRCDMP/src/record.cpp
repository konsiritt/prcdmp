
//  main.cpp
//  PRCDMP
//
//  Created by Elie Aljalbout on 04.10.18.
//  Copyright © 2018 Elie Aljalbout. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <string>
#include "DMP/DMP.hpp"
#include "UTILS/Config.h"
#include "UTILS/common.h"
#include "DMP/DiscreteDMP.hpp"
#include "UTILS/trajectoryUtils.h"
#include <math.h>
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
    std::vector<std::vector<double>> w ;

    parseArgs(argc, argv);

    std::string datasetPath = argv[1];
    // handles config file access
    Config config(datasetPath);
    //fill data from json to variables
    int dofs = config.getDmpJson()["dofs"].asInt();
    double timeSpan = config.getDmpJson()["timespan"].asDouble();
    // initialize arrays from config file
    std::array<double,7> q0;
    moveJsonArrayToVec(config.getDmpJson()["q0"], q0);

    //fill data from json to variables
    std::string robotIp = config.getDataJson()["robot_ip"].asString();
    std::cout<<"Robot ip : "<<robotIp<<std::endl;
    int episodeNr = config.getDataJson()["current_episode"].asInt();
    std::cout<<"ep num "<<episodeNr<<std::endl;
    config.fillTrajectoryPath(episodeNr);

    Interface roboInter;
    std::array<double,7> finalQ;
    std::cout<<"config.getTrajectoryPath() "<<config.getTrajectoryPath()<<std::endl;
    if(roboInter.recordTrajectory(robotIp, config.getTrajectoryPath(), q0, (float)timeSpan, finalQ))
    {
        //save to json
        std::ofstream datafwriter(config.getDataConfPath());
        // save goal to config file
        for (int i=0; i<dofs;i++)
        {
            config.getDmpJson()["goal"][i]=finalQ[i];
        }
        Json::Value dataJson = config.getDataJson();
        dataJson["current_episode"] = episodeNr+1;
        Json::StyledWriter styledWriter, styledWriter2;
        if (episodeNr==0)
        {
            std::cout<<"Episode number: "<<episodeNr<<std::endl;
            std::ofstream dmpfwriter(config.getDmpConfPath());
            dmpfwriter<<styledWriter.write(config.getDmpJson());
            dmpfwriter.close();
        }

        datafwriter<<styledWriter2.write(dataJson);
        datafwriter.close();

        std::cout<<"End of Record"<<std::endl;
        return 0;
    }
    else
    {
        return 1;
    }
}

