# -*- coding: utf-8 -*-
from scipy.integrate import odeint
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
from cProfile import label


class SEIR:

    def __init__(self,
                 incubation_period=5.5,
                 infective_period=7,
                 basic_reproduction_rate=2,
                 intervention_times=[70, 75, 100, 200, 300],
                 p0=(83019212., 0., 1., 0.),
                 t_vals=np.arange(0., 365., 1.)):
        self.t_inf = infective_period
        self.t_inc = incubation_period
        self.r0 = basic_reproduction_rate
        self.t_vals = t_vals
        self.p0 = tuple(p0)
        self.n = sum(self.p0)
        self.intervention_times = intervention_times

    def get_current_r(self, t, r_list):
        current_r = r_list[0]
        for i, intervention_time in enumerate(self.intervention_times):
            if t >= intervention_time:
                if (i + 1) < len(r_list):
                    current_r = r_list[i + 1]
        return current_r

    def dS(self, t, susceptible, infectious, r_list):
        current_r = self.get_current_r(t, r_list)
        return -1 * susceptible / self.n * (current_r / self.t_inf * infectious)

    def dE(self, t, susceptible, exposed, infectious, r_list):
        current_r = self.get_current_r(t, r_list)
        return (susceptible / self.n) * (current_r / self.t_inf * infectious) - exposed / self.t_inc

    def dI(self, exposed, infectious):
        return exposed / self.t_inc - infectious / self.t_inf

    def dR(self, infectious):
        return infectious / self.t_inf

    def system(self, y, t, r_list=None):
        susceptible, exposed, infectious, removed = y
        if r_list is None:
            r_list = [self.r0] * len(self.intervention_times)
        return [self.dS(t, susceptible, infectious, r_list),
                self.dE(t, susceptible, exposed, infectious, r_list),
                self.dI(exposed, infectious),
                self.dR(infectious)]

    def __call__(self, t, r0, r1, r2, r3, e0):
        return np.sum(self.getSEIR(t, [r0, r1, r2, r3, r1, r0], e0)[:, 1:], axis=1)

    def getSEIR(self, t, r_list, e0):
        S0, E0, I0, R0 = self.p0
        p0 = (self.n - E0 - I0 - E0, e0, I0, R0)
        return odeint(self.system, y0=p0, t=t, args=(r_list,))


if __name__ == "__main__":
    Sres = []
    Eres = []
    Ires = []
    Rres = []
    bev_de = 83019213.
    times = np.arange(0., 2 * 365., 1.0)
    inter_times = [70, 75, 100, 125, 150, 175, 200, 225, 250, 275]
    rep_values = [3.4, 2.5, 1.15, 0.8, 1.15, 0.8, 1.15, 0.8, 1.15, 0.8]
    e0 = 0.
    p = [bev_de - 1, e0, 1., 0.]  # Start condition for S, E, I, R
    model = SEIR(p0=p, intervention_times=inter_times, t_vals=times)
    for p in model.getSEIR(times, rep_values, e0):
        # print("{0:09.0f}\t{1:09.0f}\t{2:09.0f}\t{3:09.0f}".format(*p))
        Sres.append(p[0])
        Eres.append(p[1])
        Ires.append(p[2])
        Rres.append(p[3])
        
    fig, ax = plt.subplots()
    #ax.plot(times, Sres, label='S')
    ax.plot(times, Eres, label='E')
    ax.plot(times, Ires, label='I')
    ax.plot(times, Rres, label='R')
    ax.set(xlabel='time (days)', ylabel='people', title='Plot SEIR model')
    ax.grid()
    ax.legend()
    
    #fig.savefig("seirmodel.png")
    plt.show()
    
