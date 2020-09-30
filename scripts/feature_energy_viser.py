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

    def extract_diff_value(line):
        res = re.findall("diff = (.*?)weighted", line)
        if len(res) == 0:
            return None
        else:
            # print(line)
            res = res[0].split()
            num = np.array([float(i) for i in res])
            return np.linalg.norm(num)

    def extract_weight_diff_value(line):
        res = re.findall("weighted diff = (.*?)$", line)
        if res is None:
            print(f" not found {line}")
            exit()
            return None
        else:
            res = res[0].split()
            num = np.array([float(i) for i in res])
            return np.linalg.norm(num)

    def extract_feature_energy_value(line):
        return float(line.split()[-1])

    def extract_title(line):
        name = str(line.split()[1])
        return name

    for line in cont:
        if line.find("[fea]") != -1:
            title = extract_title(line)
            diff_value = extract_diff_value(line)

            weight_diff_value = None
            if diff_value is None:
                diff_value = extract_feature_energy_value(line)

            else:
                weight_diff_value = extract_weight_diff_value(line)
                # print(weight_diff_value)
                # exit()

            if title not in energy_term_dict.keys():
                energy_term_dict[title] = []
            energy_term_dict[title].append(diff_value)

            if weight_diff_value is not None:
                weighted_title = title + "_weight"
                if weighted_title not in energy_term_dict.keys():
                    energy_term_dict[weighted_title] = []
                energy_term_dict[weighted_title].append(weight_diff_value)

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
    feature_energy_dict, num_frames = load_info("../feature_energy.txt")

    if num_frames != prev_frames:
        # get the layout
        num = 1
        for i in feature_energy_dict.keys():
            if i[-6:] != "weight":
                num += 1

        width = 2
        height = 3

        while num > width * height:
            width += 1
            height += 1
        weighted_value_lst = []
        weighted_value_label_lst = []
        idx = 0
        for key in feature_energy_dict.keys():
            if key[-6:] == "weight":
                weighted_value_lst.append(lst[-1])
                weighted_value_label_lst.append(key)
                continue

            lst = feature_energy_dict[key]
            plt.subplot(width, height, idx + 1)
            idx += 1
            plt.cla()
            plt.plot(lst)
            plt.title(key)

        print(f"num {num}")
        plt.subplot(width, int(height/2), width * int(height/2))
        plt.cla()
        weighted_value_lst /= np.sum(weighted_value_lst)

        plt.pie(weighted_value_lst, labels=weighted_value_label_lst)
    plt.tight_layout()
    plt.draw()
    plt.pause(0.1)
