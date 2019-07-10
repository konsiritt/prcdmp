'''
Copyright (C) 2013 Travis DeWolf

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


from pydmps.cs import CanonicalSystem


class DMPs(object):
    """Implementation of Dynamic Motor Primitives,
    as described in Dr. Stefan Schaal's (2002) paper."""

    def __init__(self, n_dmps, n_bfs, dt=.001,
                 y0=0, goal=1, w=None,
                 ay=None, by=None, **kwargs):
        """
        n_dmps int: number of dynamic motor primitives
        n_bfs int: number of basis functions per DMP
        dt float: timestep for simulation
        y0 list: initial state of DMPs
        goal list: goal state of DMPs
        w list: tunable parameters, control amplitude of basis functions
        ay int: gain on attractor term y dynamics
        by int: gain on attractor term y dynamics
        """

        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt
        if isinstance(y0, (int, float)):
            y0 = np.ones(self.n_dmps)*y0
        self.y0 = y0
        if isinstance(goal, (int, float)):
            goal = np.ones(self.n_dmps)*goal
        self.goal = goal
        if w is None:
            # default is f = 0
            w = np.zeros((self.n_dmps, self.n_bfs))
        self.w = w

        self.gain_a = np.ones(n_dmps) * 25. if ay is None else np.array(ay)  # Schaal 2012
        self.gain_b = self.gain_a / 4. if by is None else np.array(by)  # Schaal 2012 np.sqrt(self.gain_a)*2

        # set up the CS
        self.canonical_system = CanonicalSystem(dt=self.dt, **kwargs)
        self.timesteps = int(self.canonical_system.run_time / self.dt)

        # set up the DMP system
        self.reset_state()

    def check_offset(self):
        """Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0"""

        for d in range(self.n_dmps):
            if (self.y0[d] == self.goal[d]):
                self.goal[d] += 1e-4

    def gen_front_term(self, x, dmp_num):
        raise NotImplementedError()

    def gen_goal(self, y_des):
        raise NotImplementedError()

    def gen_psi(self):
        raise NotImplementedError()

    def gen_weights(self, f_target):
        raise NotImplementedError()

    def imitate_path(self, y_des, plot=False):
        """Takes in a desired trajectory and generates the set of
        system parameters that best realize this path.

        y_des list/array: the desired trajectories of each DMP
                          should be shaped [n_dmps, run_time]
        """

        # set initial state and goal
        if y_des.ndim == 1:
            y_des = y_des.reshape(1, len(y_des))
        self.y0 = y_des[:, 0].copy().astype(np.float)
        self.y_des = y_des.copy()
        self.goal = self.gen_goal(y_des)

        self.check_offset()

        # generate function to interpolate the desired trajectory
        import scipy.interpolate
        print("self.timesteps" ,self.timesteps)
        path = np.zeros((self.n_dmps, self.timesteps))
        x = np.linspace(0, self.canonical_system.run_time, y_des.shape[1])

        #Normalizing the input trajectory in time (squashing it from [0-T] to [0-1])
        for d in range(self.n_dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[d])
            for t in range(self.timesteps):
                path[d, t] = path_gen(t * self.dt)
        y_des = path

        # calculate velocity of y_des
        dy_des = np.diff(y_des) / self.dt
        # add zero to the beginning of every row
        dy_des = np.hstack((np.zeros((self.n_dmps, 1)), dy_des))

        # calculate acceleration of y_des
        ddy_des = np.diff(dy_des) / self.dt
        # add zero to the beginning of every row
        ddy_des = np.hstack((np.zeros((self.n_dmps, 1)), ddy_des))

        f_target = np.zeros((y_des.shape[1], self.n_dmps))
        # find the force required to move along this trajectory
        x_track = self.canonical_system.rollout()#(tau=1/(len(y_des[0])*self.dt))
        print("track size", len(x_track))
        for d in range(self.n_dmps):
            f_target[:, d] = ddy_des[d]/(self.gain_a[d]*self.gain_b[d])    \
                                - (float(self.goal[d]) - y_des[d])   \
                                + dy_des[d]/self.gain_b[d]   \
                                + (float(self.goal[d]) - self.y0[d])*x_track

                #(ddy_des[d] - self.gain_a[d] *
                 #             (self.gain_b[d] * (float(self.goal[d]) - y_des[d]) -
                  #            dy_des[d]))

        # efficiently generate weights to realize f_target
        self.gen_weights(f_target)

        if plot is True:
            # plot the basis function activations
            import matplotlib.pyplot as plt
            plt.figure()
            plt.subplot(211)
            psi_track = self.gen_psi(self.canonical_system.rollout())
            print(psi_track.shape)
            plt.plot(psi_track)
            plt.title('basis functions')

            # plot the desired forcing function vs approx
            plt.subplot(212)
            plt.plot(f_target[:,0])
            plt.plot(np.sum(psi_track * self.w[0], axis=1) * self.dt)
            plt.legend(['f_target', 'w*psi'])
            plt.title('DMP forcing function')
            plt.tight_layout()
            plt.show()

        self.reset_state()
        return y_des

    def rollout(self, timesteps=None, **kwargs):
        """Generate a system trial, no feedback is incorporated."""


        self.reset_state()

        if timesteps is None:
            if 'tau' in kwargs:
                print("kwargs['tau']",kwargs['tau'])
                timesteps = int(self.timesteps / kwargs['tau'])
            else:
                timesteps = self.timesteps

        print(timesteps, "timesteps")
        print("goal", self.goal)
        print("y0", self.y0)
        # set up tracking vectors
        y_track = np.zeros((timesteps, self.n_dmps))
        dy_track = np.zeros((timesteps, self.n_dmps))
        ddy_track = np.zeros((timesteps, self.n_dmps))

        for t in range(timesteps):

            # run and record timestep
            y_track[t], dy_track[t], ddy_track[t] = self.step(**kwargs)
            print("dy_track",dy_track[t])

        return y_track, dy_track, ddy_track

    def reset_state(self):
        """Reset the system state"""
        self.y = self.y0.copy()
        self.dy = np.zeros(self.n_dmps)
        self.ddy = np.zeros(self.n_dmps)
        self.canonical_system.reset_state()

    def step(self, tau=1.0, error=0.0, external_force=None):
        """Run the DMP system for a single timestep.0

        tau float: scales the timestep
                   increase tau to make the system execute faster
        error float: optional system feedback
        """

        error_coupling = 1.0 / (1.0 + error)
        # run canonical system
        x = self.canonical_system.step(tau=tau, error_coupling=error_coupling)
        #print("x : ", x)
        # generate basis function activation
        psi = self.gen_psi(x)

        for d in range(self.n_dmps):
            # generate the forcing term
            #print("x : ",x)
            #print("d : ",d)
            #print("self.gen_front_term(x, d) : ",self.gen_front_term(x, d))
            #print("psi : ",psi)
            #print("self.w[d] : ",self.w[d])
            #print("(np.dot(psi, self.w[d])) : ",(np.dot(psi, self.w[d])))
            #print("np.sum(psi) : ",np.sum(psi))
            f = (x*(np.dot(psi, self.w[d])) / np.sum(psi)) #changed
            #print("f :",f)
            # DMP acceleration
            #print("self.gain_a[d] : ",self.gain_a[d])
            #print("self.gain_b[d] : ", self.gain_b[d])
            #print("tau : ",tau)
            self.ddy[d] = (tau * tau * self.gain_a[d] *
                           (self.gain_b[d] * (self.goal[d] - self.y[d] - (self.goal[d]-self.y0[d])*x + f) - self.dy[d]/tau)) #
            #print("self.ddy[d] :",self.ddy[d])
            if external_force is not None:
                self.ddy[d] += external_force[d]
            self.dy[d] += self.ddy[d] * self.dt * error_coupling
            #print(" self.dy[d] : ", self.dy[d])
            self.y[d] += self.dy[d] * self.dt * error_coupling
            #print("self.y[d] :", self.y[d])

        return self.y, self.dy, self.ddy
