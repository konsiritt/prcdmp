#ifndef INTERFACE_H
#define INTERFACE_H

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <string>
#include "DMP/DMP.hpp"
#include "UTILS/trajectoryUtils.h"

#include <common.h>

class Interface
{
public:
    Interface();

    bool recordTrajectory(std::string ipRobot, std::string fileName, std::array<double,7> q0, float recordDuration, std::array<double,7> &finalQ);

    void runDMPTrajectory(DMP &dmp, std::string ipRobot, std::vector<double> externalForce, std::array<double, 7> &qGoal, int timesteps = -1, double tau=1.0);

    int runTrajectory(std::string ipRobot, std::array<double, 7> &q0, std::string trajectoryPath, int dofs);

    void movePointToPoint(char* ip,std::array<double,7> qGoal,double speed, std::array<double,13> load);

    void setDefaultBehavior(franka::Robot& robot);

private:



};

#endif // INTERFACE_H
