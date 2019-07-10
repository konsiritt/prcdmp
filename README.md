# prcdmp
DMP formulation after "Learning and generalization of motor skills by learning from demonstration", Pastor et. al 2009 
(https://ieeexplore.ieee.org/abstract/document/5152385)

## configuration
Trajectories are organized in folders for each trajectory in *data/*. In *data/nameOfTrajectory/conf/dataset.json* you can configure the following:

Field     | Description|
-------- | ---
robot_ip| IP of the robot, needs to be reachable by the machine running the code
current_episode | recording a new trajectory in the same folder will increment this counter

DMP specific configuration is handled in *data/nameOfTrajectory/conf/DMP.json*. Important fields are the following:

Field     | Description|
-------- | ---
q0| joint space initial configuration of the robot
goal | joint space goal configuration of the robot (will be adapted accordingly after recording
n_basis | amount of basis functions to approximate the trajectory
timespan | duration of trajectory for recording in [s]
dofs | degrees of freedom
dt | timestep size in [s]

## usage
### recording trajectories

The script record.sh is for recording a new trajectory with the robot specified in conf.json. 
Call `./record.sh <nameOfTrajectory> w` to record a trajectory with the specified name. The *w* issues the weight fitting, without it, only the trajectory will be saved.

### replaying trajectories

The relevant script is rundmp.sh. Again the name of the trajectory you want to run needs to be specified. Also the *current_episode* needs to be set accordingly (always one higher than what you want to see, so set it to 1 if you want to run 0, yes, weird, but a leftover I did not bother fixing)
Type `./rundmp.sh <nameOfTrajectory> r` for the trajectory to be executed on the robot. 
