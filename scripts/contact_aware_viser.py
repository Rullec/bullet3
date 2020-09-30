import os
import numpy as np
import re
import time
import matplotlib.pyplot as plt
# 1. load current info


def load_info(path):
    ref_traj_info = {"q": [], "qdot": [], "qddot": []}
    fbf_traj_info = {"q": [], "qdot": [], "qddot": []}
    ctrl_res_info = {"q": [], "qdot": [], "qddot": []}
    contact_force_lst = []
    control_force_lst = []
    contact_num_lst = []

    with open(path, 'r') as f:
        cont = [line.strip() for line in f.readlines()]

    def extract_numbers(line):
        return [float(i) for i in line.split()[4:]]
    for line in cont:
        if line.find("[numeric] ref q =") != -1:
            ref_traj_info["q"].append(extract_numbers(line))
        elif line.find("[numeric] ref qdot =") != -1:
            ref_traj_info["qdot"].append(extract_numbers(line))
        elif line.find("[numeric] ref qddot =") != -1:
            ref_traj_info["qddot"].append(extract_numbers(line))
        elif line.find("[numeric] ctrl_res q =") != -1:
            ctrl_res_info["q"].append(extract_numbers(line))
        elif line.find("[numeric] ctrl_res qdot =") != -1:
            ctrl_res_info["qdot"].append(extract_numbers(line))
        elif line.find("[numeric] ctrl_res qddot =") != -1:
            ctrl_res_info["qddot"].append(extract_numbers(line))
        elif line.find("[numeric] FBF q =") != -1:
            fbf_traj_info["q"].append(extract_numbers(line))
        elif line.find("[numeric] FBF qdot =") != -1:
            fbf_traj_info["qdot"].append(extract_numbers(line))
        elif line.find("[numeric] FBF qddot =") != -1:
            fbf_traj_info["qddot"].append(extract_numbers(line))
        elif line.find("[numeric] contact force =") != -1:
            contact_force_lst.append(extract_numbers(line))
        elif line.find("[numeric] control force =") != -1:
            control_force_lst.append(extract_numbers(line))
        elif line.find("[debug] contact_pts num =") != -1:
            contact_num_lst.append(extract_numbers(line))
            # print(extract_numbers(line))

    frame_num = len(ref_traj_info["q"])
    print(f"num of frames {frame_num}")
    return ref_traj_info, fbf_traj_info, ctrl_res_info, contact_force_lst, control_force_lst, contact_num_lst


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
    ref_traj_info, fbf_traj_info, ctrl_res_info, contact_force_lst, control_force_lst, contact_num_lst = load_info(
        "../numeric.log")
    # print(ref_traj_info["q"])
    # exit()
    fbf_err_lst = [np.linalg.norm(np.array(ref_traj_info["q"][i]) - np.array(fbf_traj_info["q"][i]))
                   for i in range(len(ref_traj_info["q"]))]
    contact_aware_err_lst = [np.linalg.norm(np.array(ctrl_res_info["q"][i]) - np.array(fbf_traj_info["q"][i]))
                             for i in range(len(ref_traj_info["q"]))]
    total_err_lst = [np.linalg.norm(np.array(ctrl_res_info["q"][i]) - np.array(ref_traj_info["q"][i]))
                     for i in range(len(ref_traj_info["q"]))]
    # print(np.linalg.norm(fbf_err_lst))
    # print(np.linalg.norm(contact_aware_err_lst))
    # print(np.linalg.norm(total_err_lst))
    # exit(0)
    now_frames = len(control_force_lst)
    # if now_frames != prev_frames:
    if True:
        prev_frames = now_frames

        plt.subplot(3, 5, 1)
        plot_value(ref_traj_info["q"], "ref_q", extract_root)
        plt.subplot(3, 5, 6)
        plot_value(ctrl_res_info["q"], "ctrl_res_q", extract_root)
        plt.subplot(3, 5, 11)
        plot_value(fbf_traj_info["q"], "fbf_q", extract_root)

        plt.subplot(3, 5, 2)
        plot_value(ref_traj_info["qdot"], "ref_qdot", extract_root)
        plt.subplot(3, 5, 7)
        plot_value(ctrl_res_info["qdot"], "ctrl_res_qdot", extract_root)
        plt.subplot(3, 5, 12)
        plot_value(fbf_traj_info["qdot"], "fbf_qdot", extract_root)

        plt.subplot(3, 5, 3)
        plot_value(ref_traj_info["qddot"], "ref_qddot", extract_root)
        plt.subplot(3, 5, 8)
        plot_value(ctrl_res_info["qddot"], "ctrl_res_qddot", extract_root)
        plt.subplot(3, 5, 13)
        plot_value(fbf_traj_info["qddot"], "fbf_qddot", extract_root)

        # plt.subplot(2, 5, 1)
        # plot_value(ref_traj_info["q"], "ref_q", extract_root)

        # plt.subplot(2, 5, 3)
        # plot_value(ref_traj_info["qddot"], "ref_qddot", extract_root)

        # plt.subplot(2, 5, 6)
        # plot_value(ctrl_res_info["q"], "ctrl_res_q", extract_root)

        # plt.subplot(2, 5, 8)
        # plot_value(ctrl_res_info["qddot"], "ctrl_res_qddot", extract_root)
        plt.subplot(3, 5, 4)
        plot_value(contact_num_lst, "ctrl_res_contact_num", extract_norm)
        plt.subplot(3, 5, 9)
        plot_value(contact_force_lst, "ctrl_res_contact_force", extract_norm)
        plt.subplot(3, 5, 14)
        plot_value(control_force_lst, "ctrl_res_control_force", extract_norm)

        plt.subplot(3, 5, 5)
        plt.cla()
        plt.plot(fbf_err_lst, label="fbf_err")
        plt.plot(contact_aware_err_lst, label="contact_aware_err")
        plt.plot(total_err_lst, label="total_err")
        plt.title("3 control err")
        plt.legend()

    # flush
    plt.draw()
    plt.pause(0.1)

    # keep
    # plt.show()
