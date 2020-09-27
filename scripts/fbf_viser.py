import os
import numpy as np
import re
import time
import matplotlib.pyplot as plt
# 1. load current info


def load_info(path):
    energy_term_dict = {}
    with open(path, 'r') as f:
        cont = [line.strip() for line in f.readlines()]

    def extract_value(line):
        return float(line.split()[5][:-1])

    def extract_coef(line):
        return float(line.split()[8])

    def extract_title(line):
        return str(line.split()[2])

    for line in cont:
        if line.find("[energy]") != -1:
            title = extract_title(line)
            coef = extract_coef(line)
            value = extract_value(line)
            if title not in energy_term_dict.keys():
                energy_term_dict[title] = []
            energy_term_dict[title].append(value)

    frames = max([len(energy_term_dict[i])
                  for i in energy_term_dict.keys()] + [0])

    print(f"num of frames {frames}")
    return energy_term_dict, frames


def plot_value(content, title, func):
    plt.cla()
    val_lst = []
    for line in content:
        # norm_lst.append(np.linalg.norm(line[0:3]))
        val_lst.append(func(line))
    # print(val_lst)
    plt.plot(val_lst)
    plt.title(title)
    return


# 2. disply & calculate


def extract_root(x): return x[1]
def extract_norm(x): return np.linalg.norm(x)


# plt.ion()
prev_frames = -1
while True:
    energy_term_dict, num_frames = load_info("../energy.txt")

    if num_frames != prev_frames:
        # get the layout
        num = len(energy_term_dict.keys())
        width = np.sqrt(num)
        if float(int(width)) != width:
            width = int(width) + 1
        else:
            width = int(width)

        for idx, key in enumerate(energy_term_dict.keys()):
            lst = energy_term_dict[key]
            plt.subplot(width, width, idx + 1)
            plt.cla()
            plt.plot(lst)
            plt.title(key)
    plt.draw()
    plt.pause(0.1)
    # plt.show()
