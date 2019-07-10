'''
Copyright (C) 2016 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("..")

from util.read_franka_traj import *
import seaborn

import pydmps
import pydmps.dmp_discrete
import json
import argparse
import os.path


def main(dataset_path, learn, plot):

    if dataset_path[-1]== "/":
        dataset_path = dataset_path[:-1]
    if dataset_path[0] == os.sep:
        dataset_path = dataset_path[1:]
    dmp_conf_path = "../../data/" + dataset_path + "/conf/DMP.json"
    data_conf_path = "../../data/" + dataset_path + "/conf/dataset.json"


    with open(data_conf_path, "r") as dataf:
        dataset_data = json.load(dataf)
        episode_nr = int(dataset_data["current_episode"])
        recording_path = "../../data/" + dataset_path + "/episodes/episode"+str(episode_nr-1)+"/recording.csv"
        replay_path = "../../data/" + dataset_path + "/episodes/episode" + str(episode_nr-1) + "/replay.csv"
        if episode_nr>1:
            weights_path = "../../data/" + dataset_path + "/episodes/episode"+str(episode_nr-1)+"/weights.csv"
        else:
            weights_path = "../../data/" + dataset_path + "/conf/weights.csv"

    if not os.path.isfile(dmp_conf_path):
        print("The given configuration file does not exist. \nHINT: check if given dataset path is correct")
        exit(62)

    if not os.path.isfile(recording_path):
        print("The given recording file does not exist. \nHINT: check if given dataset path is correct\n")
        print(recording_path)
        exit(63)

    with open(dmp_conf_path,"r+") as f:
        data = json.load(f)
        dofs        = data["dofs"]
        nbs         = data["n_basis"]
        dt          = data["dt"]
        gainA       = data["gain_a"]
        timeSpan    = data["timespan"]


        y_des = read_traj(recording_path)
        dim= len(y_des)
        if dim!=dofs:
            print("Expected DOFs are different from recorded one")
            exit(64)

        dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=dim, n_bfs=1000, dt=dt, ay=gainA)#np.ones(7)*10)
        y_track = []
        dy_track = []
        ddy_track = []

        dmp.imitate_path(y_des=y_des, plot=plot)


        if episode_nr<=1:
            for idx, y0elem in enumerate(dmp.y0):
                data["q0"][idx] = y0elem
                data["goal"][idx] = dmp.goal[idx]
            f.seek(0)  # rewind
            json.dump(data, f, indent=4)
            f.truncate()
        os.makedirs(os.path.abspath(os.path.join(weights_path, os.pardir)), exist_ok=True)
        np.savetxt(weights_path, dmp.w, delimiter=',')

    if episode_nr>1:
        dmp.goal = data["goal"]
    y_track, dy_track, ddy_track = dmp.rollout(tau=1/timeSpan)
    track = np.concatenate((y_track,dy_track,ddy_track), axis=1)
    np.savetxt(replay_path, track, delimiter=',')
    print("after")
    if plot:
        plt.figure(1, figsize=(6,6))
        plot_dim = 5
        #print(y_track[:,plot_dim], len(y_track[:,plot_dim]))
        #print(y_des[plot_dim,:], len(y_des[plot_dim,:]))
        t_track = [t*0.001 for t in range(len(y_track[:,plot_dim]))]
        #np.savetxt("out/generated.txt", y_track, delimiter = ',')
        plt.plot(t_track, y_track[:,plot_dim], 'o', color= 'green')
        #np.savetxt("pythondys.txt",dy_track, delimiter = ',')

        des_plot = y_des[plot_dim,:]
        des_plot = des_plot[::5]
        #np.savetxt("desired6.txt", np.array(des_plot).astype(np.float), delimiter = ',')
        #print(len(des_plot))
        t_track2 = [t*0.001 for t in range(len(des_plot))]
        plt.plot(t_track2, des_plot, 'o', color = 'red')
        plt.title('DMP Output in Dimension: '+str(plot_dim))

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    requiredArgs = parser.add_argument_group('required arguments')
    requiredArgs.add_argument("-d", "--dataset", help="specifies the relative pass of the dataset to use starting from prcdmp/data",
                              required=True)

    parser.add_argument("--learn", action='store_true',
                        help="Use it if you want to learn (imitate) a recording and save the weights")
    parser.add_argument("--visualize", action='store_true',
                        help="Use it if you want to have plots about the final results and dmp weights")
    args = parser.parse_args()
    # Train/Visualize as per the arguments
    dataset_path = args.dataset

    if(len(dataset_path)==0):
        print("Please provide a dataset path")
        exit(69)

    main(dataset_path, args.learn, args.visualize)



