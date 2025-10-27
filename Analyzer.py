import os

import matplotlib.pyplot as plt

from monkey_bot.simulation_config import SimConfig
if __name__ == "__main__":
    data_file = open("adjust_angle_data.txt").read()
    desired_rates = []
    real_rates = []
    new_rates = []

    for l in data_file.split("\n"):
        if len(l)==0:
            break
        l = l.split(', ')
        des, real, new = [(float(x) if x == "None" else 0) for x in l ]
        desired_rates.append(des)
        real_rates.append(real)
        new_rates.append(new)
    desired_rates = desired_rates
    real_rates = real_rates
    new_rates = new_rates
    plt.plot(range(len(desired_rates)), desired_rates)
    plt.plot(range(len(desired_rates)), real_rates)
    plt.plot(range(len(new_rates)), new_rates)
    plt.legend(['Desired', 'Real', 'Sent signal'])


    plt.show()