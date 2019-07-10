#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include "UTILS/common.h"

class Config
{
public:
    Config(std::string datasetPath);

    void fillTrajectoryPath(int episodeNr);

    // Getters
    Json::Value getDmpJson();
    Json::Value getDataJson();
    std::string getwPath();
    std::string getInitialWPath();
    std::string getTrajectoryPath();
    std::string getReplayTrajectoryPath();
    std::string getReplayTrajectoryPathQ();
    std::string getDmpConfPath();
    std::string getDataConfPath();

private:

    std::string datasetPath;
    std::string basePath;
    std::string confBasePath;
    std::string dmpConfPath;
    std::string dataConfPath;
    std::string trajectoryPath;
    std::string replayTrajectoryPath;
    std::string replayTrajectoryPathQ;
    std::string wPath;
    std::string initialWPath;

    std::ifstream dmpconf;
    std::ifstream dataconf;

    Json::Reader dmpConfReader;
    Json::Reader dataConfReader;

    Json::Value dmpJson;
    Json::Value dataJson;
};

#endif // CONFIG_H
