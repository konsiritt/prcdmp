//
// Created by Elie Aljalbout on 08.10.18.
//

#include "UTILS/trajectoryUtils.h"

using namespace std;

namespace UTILS{

    void split(const string & s, char c,vector<double>& v) {
        int i = 0;
        int j = (int) s.find(c);
        double tmp;
        while (j >= 0) {
            tmp = stod(s.substr(i, j-i));
            v.push_back(tmp);
            i = ++j;
            j = (int) s.find(c, j);

            if (j < 0) {
                v.push_back(stod(s.substr(i, s.length())));
            }
        }
    }


    void loadSeparately(istream& in, vector<vector<double>> & data, vector<double> & times,  char separator)
    {
        vector<double> p;
        string tmp;

        //used for when no time is available from input file
        double tempTime = 0;
        double timeStep = 0.001; //corresponding to the control time step 1ms

        while (!in.eof())
        {
            std::getline(in, tmp, '\n'); // Grab the next line
            p.clear();

            split(tmp, separator, p); // Use split from
            if (p.size()==10) // input is time,cartesian_position,_velocit,_acceleration
            {
                times.push_back(p[0]);
                p.erase(p.begin());
                data.push_back(p);

            } else if (p.size()==14 || p.size()==7 || p.size()==21) // input is 7 DOF joint space (position & velocities & acceleration)
            {
                //times vector from fixed control timestep
                tempTime+=timeStep;
                times.push_back(tempTime);

                data.push_back(p);

            } else
            {
                continue;
            }

            tmp.clear();
        }
    }

    /**
     * @brief Loads a trajectory from a specified file based on separators and specified variables order
     * @param[in] fileName name of the file in which the trajectory is in
     * @param[in] order array to specify in which order the variable of each data points are written in the file
     * @param[in] innerSeparator specify the seperator of variables within each data point
     * @return a boolean specifying the success of the whole operation
     */
    bool loadTrajectory(const string & fileName, vector<vector<double>> & XYZdata,
                                 vector<double> & times, char innerSeparator)
    {
        //open file
        ifstream in(fileName);
        if (!in)
            return false;

        loadSeparately(in, XYZdata, times, ',');
        std::cout<<"returning true in loadTrajectory"<<std::endl;
        return true;
    }


    /**
    * @brief Load DMP weights from fileName and stores them into data
    * @param[in] fileName of the weights
    * @param[out] vector in which the weights will be stored
    *
    */
    bool loadWeights(const std::string & fileName, std::vector<std::vector<double>> & data)
    {
        ifstream in(fileName);

        if (!in)
            return false;

        vector<double> p; // temporary vector in which separated data will be stored in each iteration(corresponding to a line)
        string tmp;  //Temporary string in which lines will be stored in each iteration

        while(!in.eof())
        {
            std::getline(in, tmp, '\n'); // Grab the next line
            p.clear();

            split(tmp, ',', p); //split string into vector via commas
            data.push_back(p);
        }
    }


    void printData(std::vector<std::vector<double>> data)
    {
        for (auto line = data.begin(); line != data.end(); ++line)
        {
            for (vector<double>::iterator p = line->begin(); p != line->end(); ++p)
            {
                std::cout << *p << ",";
            }
            std::cout << std::endl;
        }
    }

    void writeTrajToText(const std::vector<std::vector<double>> &traj, std::string file_name)
    {
        std::ofstream myfile;
        myfile.open(file_name);
        for (int i = 0; i < traj.size(); i++)
        {
            for (int j = 0; j < traj[i].size(); j++)
            {
                if (j == traj[i].size()-1)
                {
                    myfile << traj[i][j];
                }
                else {
                    myfile << traj[i][j] << ',' ;
                }

            }
            myfile <<std::endl;
        }
        myfile.close();
    }

    void writeTrajTimeToText(const std::vector<std::vector<double>> &traj, std::vector<double> &time, std::string file_name)
    {
        std::ofstream myfile;
        myfile.open(file_name);
        for (int i = 0; i < traj.size(); i++)
        {
            myfile << time[i] << ',';
            for (int j = 0; j < traj[i].size(); j++)
            {
                if (j == traj[i].size()-1)
                {
                    myfile << traj[i][j];
                }
                else {
                    myfile << traj[i][j] << ',' ;
                }

            }
            myfile <<std::endl;
        }
        myfile.close();
    }


}

