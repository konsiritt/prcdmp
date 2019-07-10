#include "interface.h"

Interface::Interface()
{

}

void Interface::setDefaultBehavior(franka::Robot& robot)
{
    robot.setCollisionBehavior(
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    robot.setFilters(1000, 1000, 1000, 1000, 1000);
}

void Interface::movePointToPoint(char* ip,std::array<double,7> qGoal,double speed, std::array<double,13> load)
{
    try
    {
        franka::Robot robot(ip); // initialization of the robot object passing the robot IP to its constructor function
        setDefaultBehavior(robot);

        // Set external load for the manipulator ========================================================
        robot.setLoad(load[0],{load[1],load[2],load[3]},{load[4],load[5],load[6],load[7],load[8],load[9],load[10],load[11],load[12]});
        // ==============================================================================================

        double speed_factor = speed;

        // Set collision behavior.
        robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

        MotionGenerator motion_generator(speed_factor, qGoal);
        robot.control(motion_generator);
        std::cout << "Motion finished" << std::endl;

    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return;
    }
}

bool Interface::recordTrajectory(std::string ipRobot, std::string fileName, std::array<double,7> q0, float recordDuration, std::array<double,7> &finalQ)
{
    std::array<double,13> load = {0,0,0,0,0,0,0,0,0,0,0,0,0}; // std::array is another way to define an array
    franka::Robot robot(ipRobot);
    robot.setLoad(load[0],{load[1],load[2],load[3]},{load[4],load[5],load[6],load[7],load[8],load[9],load[10],load[11],load[12]});

    // open text file
    std::ofstream recordFile;
    recordFile.open(fileName);

    try{
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{50.0, 50.0, 50.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{50.0, 50.0, 50.0, 25.0, 25.0, 25.0}});

        double time = 0.0, ramp = 0.0;

        // move to initial position
        movePointToPoint(&ipRobot[0u],q0,0.1,load);

        std::cout<<"after move to point"<<std::endl;
        franka::Robot robot(ipRobot);
        franka::Model model= robot.loadModel();
        robot.control([=, &time, &model, &ramp, &recordFile, &finalQ](const franka::RobotState& state,franka::Duration time_step) -> franka::Torques {

            time += time_step.toSec();
            Eigen::Matrix<double,7,1> q = Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.q).data());

            // write positions to disk
            const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

            recordFile << q.transpose().format(CSVFormat);
            recordFile << std::endl;

            franka::Torques tau = {0, 0, 0, 0, 0, 0, 0};
            // stop recording after specified number of seconds
            if (time > recordDuration)
            {
                finalQ = state.q;
                return franka::MotionFinished(tau);
            }
            return tau;
        });
    }
    catch (const franka::ControlException& e)
    {
         std::cout << e.what() << std::endl;
         std::cout << "Running error recovery..." << std::endl;
         robot.automaticErrorRecovery();
         recordFile.close();
         return false;
     }
    catch (const franka::Exception& e) {
        std::cout <<e.what() << std::endl;
        sleep(1);
        //robot.automaticErrorRecovery();
        recordFile.close();
        return false;
    }
    recordFile.close();

    return true;
}

void Interface::runDMPTrajectory(DMP &dmp, std::string ipRobot, std::vector<double> externalForce,
                              std::array<double, 7> &q0, int timesteps, double tau )
{
    try
    {
        if(timesteps ==-1)
        {
            timesteps = dmp.getTimesteps();
            //std::cout<<"amount of timesteps for current dmp: "<<timesteps<<std::endl;
        }

        if(timesteps == 0)
        {
            std::cerr<<"Timesteps set to 0"<<std::endl;
            return;
        }
        timesteps = timesteps/tau;
        //std::cout<<"amount of timesteps for current dmp: "<<timesteps<<std::endl;
        franka::Robot robot(ipRobot);
        setDefaultBehavior(robot);

        MotionGenerator motionGenerator(0.4, q0);
        robot.control(motionGenerator);

        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        double time = 0.0;
        int steps = 0;
        std::cout << "WARNING: This example will move the robot! "
                          << "Please make sure to have the user stop button at hand!" << std::endl
                          << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        robot.control(
            [&time, &dmp, &timesteps, &steps, &externalForce, &tau, &velocities](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {

              time += period.toSec();

              //velocities.dq = &dmp.step(externalForce, tau)[0];
              //std::copy_n(dmp.step(externalForce, tau).begin(), 7, velocities.dq.begin());
              //franka::JointVelocities velocities = dmp.getDY().data();
              franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
              //std::vector<double> dq = dmp.getDY();

              //franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
              /*for (int i=0;i<7;i++)
              {
                  velocities.dq[i]= dq[i];
                  //std::cout<<dq[i]<<",";
              }*/
              //sstd::cout<<std::endl;
              if (steps==timesteps) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(velocities);
              }
              steps++;
              return velocities;
            });
    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return ;
    }
}

int Interface::runTrajectory(std::string ipRobot,
                              std::array<double, 7> &q0,
                              std::string trajectoryPath,
                              int dofs)
{
    try
    {
        franka::Robot robot(ipRobot);
        setDefaultBehavior(robot);

        MotionGenerator motion_generator(0.2, q0);
        robot.control(motion_generator);

        std::vector<std::vector<double>> data;
        std::vector<double> times;
        UTILS::loadTrajectory(trajectoryPath, data, times,',');
        std::cout<<"data size "<<data.size()<<std::endl;
        robot.setCollisionBehavior(
                    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        double time = 0.0;
        int iterator = 0;


        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        robot.control(
                [=, &time, &data, &iterator](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {

                  //std::cout<<"in control loop"<<std::endl;
                  time += period.toSec();

                  int start_pt =0; // only velocities are written to file
                  if (data[0].size()==dofs*2 || data[0].size()==dofs*3) // position, velocities, (accelerations) are written to file
                  {
                        //std::cout<< "data from laoded file" << data[iterator][1] << std::endl;
                        start_pt=7;
                  }
                  franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
                  for (int i=0;i<7;i++)
                  {
                      velocities.dq[i]= data[iterator][start_pt+i];
                  }
                  /*franka::JointVelocities velocities = {{data[iterator][start_pt+0],
                                                         data[iterator][start_pt+1],
                                                         data[iterator][start_pt+2],
                                                         data[iterator][start_pt+3],
                                                         data[iterator][start_pt+4],
                                                         data[iterator][start_pt+5],
                                                         data[iterator][start_pt+6]}};*/

                  iterator++;
                  if (iterator==data.size()) {
                    std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                    return franka::MotionFinished(velocities);
                  }
                  return velocities;
                });
          }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}
