import math
import os

import matplotlib.pyplot as plt

from monkey_bot.config import SimConfig
if __name__ == "__main__":
    data_file = open("adjust_angle_data.txt").read()
    desired_rates = []
    real_rates = []
    new_rates = []
    body_rates = []

    for l in data_file.split("\n"):
        if len(l)==0:
            break
        l = l.split(', ')
        des, real, new, body = [(float(x) if x != "None" else 0) for x in l ]
        des, real, new, body  = des, real, new, body
        desired_rates.append(des)
        real_rates.append(real)
        new_rates.append(new)
        body_rates.append(body)
    desired_rates = desired_rates
    real_rates = real_rates
    new_rates = new_rates
    plt.plot(range(len(desired_rates)), desired_rates)
    plt.plot(range(len(desired_rates)), real_rates)
    plt.plot(range(len(new_rates)), new_rates)
    plt.plot(range(len(body_rates)), body_rates)
    plt.legend(['Desired', 'Real', 'Sent signal', 'Body rates'])


    plt.show()