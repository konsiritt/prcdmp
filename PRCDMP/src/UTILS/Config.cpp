#include "UTILS/Config.h"

Config::Config(std::string datasetPath)
{
    basePath = "../../data/";
    basePath = basePath + datasetPath ;
    confBasePath = basePath+ std::string("/conf/");
    dmpConfPath = confBasePath + std::string("DMP.json");
    dataConfPath = confBasePath + std::string("dataset.json");
    initialWPath = confBasePath+std::string("weights.csv");

    if (!fileExists(dmpConfPath))
    {
        std::cerr<<"The DMP config Path is not correct\nHint: check the given dataset parameter"<<std::endl;
    }

    if (!fileExists(dataConfPath))
    {
        std::cerr<<"The provided data config Path is not correct\nHint: check the given dataset parameter"<<std::endl;
    }

    if (!fileExists(initialWPath))
    {
        std::cerr<<"The weights Path is not correct\nHint: check the given dataset parameter"<<std::endl;
    }

    dmpconf = std::ifstream (dmpConfPath, std::ifstream::binary);
    dataconf = std::ifstream (dataConfPath, std::ifstream::binary);

    dmpConfReader.parse(dmpconf, dmpJson);
    dataConfReader.parse(dataconf, dataJson);
}

void Config::fillTrajectoryPath(int episodeNr)
{
    wPath = basePath + std::string("/episodes/episode")+std::string(std::to_string(episodeNr))+std::string("/weights.csv");
    trajectoryPath = basePath + std::string("/episodes/episode")+std::string(std::to_string(episodeNr))+std::string("/recording.csv");
    replayTrajectoryPath = basePath + std::string("/episodes/episode")+std::string(std::to_string(episodeNr))+std::string("/replay.csv");
    replayTrajectoryPathQ = basePath + std::string("/episodes/episode")+std::string(std::to_string(episodeNr))+std::string("/replayQ.csv");
}

//getters

Json::Value Config::getDmpJson()
{
    return dmpJson;
}

Json::Value Config::getDataJson()
{
    return dataJson;
}

std::string Config::getInitialWPath()
{
    return initialWPath;
}

std::string Config::getwPath()
{
    return wPath;
}

std::string Config::getTrajectoryPath()
{
    return trajectoryPath;
}

std::string Config::getReplayTrajectoryPath()
{
    return replayTrajectoryPath;
}

std::string Config::getReplayTrajectoryPathQ()
{
    return replayTrajectoryPathQ;
}

std::string Config::getDmpConfPath()
{
    return dmpConfPath;
}

std::string Config::getDataConfPath()
{
    return dataConfPath;
}
