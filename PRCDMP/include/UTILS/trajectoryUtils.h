//
// Created by Elie Aljalbout on 08.10.18.
//

#ifndef PROJECT_TRAJECTORYUTILS_H
#define PROJECT_TRAJECTORYUTILS_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

namespace UTILS{

    void printData(std::vector<std::vector<double>> data);

    bool loadWeights(const std::string & fileName, std::vector<std::vector<double>> & data); //@todo: move to weights util file

    bool loadTrajectory(const std::string & fileName, std::vector<std::vector<double>> & XYZdata,
                                 std::vector<double> & times, char innerSeparator);
    void writeTrajToText(const std::vector<std::vector<double> > &traj, std::string file_name);    
    void writeTrajTimeToText(const std::vector<std::vector<double>> &traj, std::vector<double> &time, std::string file_name);

    /**
     * Move to some math util file
     */
    template<typename T>
    std::vector<double> linspace(T startIn, T endIn, int numOut)
    {
        std::vector<double> linspaced;
        double start    = static_cast<double>(startIn);
        double end      = static_cast<double>(endIn);
        double num      = static_cast<double>(numOut);

        if (num == 0) { return linspaced; }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for(int i=0; i < num-1; ++i)
        {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end);

        return linspaced;
    }
}

#endif //PROJECT_TRAJECTORYUTILS_H



