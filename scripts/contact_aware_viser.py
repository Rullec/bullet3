import os
import numpy as np
import re
import time
import matplotlib.pyplot as plt

# 1. load current info


# suffix = "_oneleg_revo"
suffix = "_root_y"


def load_info(path):
    ref_traj_info = {"q": [], "qdot": [], "qddot": []}
    fbf_traj_info = {"q": [], "qdot": [], "qddot": []}
    ctrl_res_info = {"q": [], "qdot": [], "qddot": []}
    contact_force_lst = []
    control_force_lst = []
    contact_num_lst = []
    control_force_diff_lst = []
    contact_force_diff_lst = []

    with open(path, "r") as f:
        cont = [line.strip() for line in f.readlines()]

    # def extract_numbers(line):
    #     return [float(i) for i in line.split()[4:]]
    def extract_numbers(line):
        global suffix
        cont = [float(i) for i in line.split()[4:]]
        return cont

    for line in cont:
        # print(line)
        # exit()
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
        elif line.find("[log] contact_force diff_percent =") != -1:
            contact_force_diff_lst.append(extract_numbers(line))
        elif line.find("[log] control_force diff_percent =") != -1:
            control_force_diff_lst.append(extract_numbers(line))
            # print(extract_numbers(line))

    frame_num = len(ref_traj_info["q"])
    print(f"num of frames {frame_num}")
    return (
        ref_traj_info,
        fbf_traj_info,
        ctrl_res_info,
        contact_force_lst,
        control_force_lst,
        contact_num_lst,
        contact_force_diff_lst,
        control_force_diff_lst,
    )


def plot_value(content, title, func):
    plt.cla()
    val_lst = []
    for line in content:
        # norm_lst.append(np.linalg.norm(line[0:3]))
        # print(f"line {line}")
        val_lst.append(func(line))
    # print(val_lst)
    plt.plot(val_lst)
    plt.title(title)
    return


# 2. disply & calculate


def extract_root_y_value(x):
    return x[1]


def extract_oneleg_revolute_dof_value(x):
    return x[6]


def extract_oneleg_revolute_control_force(x):
    return x[0]


def extract_norm(x):
    return np.linalg.norm(x)


def principle_value_analysis(value_lst, prefix):
    assert type(value_lst) is list
    assert type(value_lst[0]) is list
    assert len(value_lst[0]) is 1
    value_lst = [i[0] for i in value_lst]
    
    value_lst = sorted(value_lst)
    # print(value_lst)
    # exit(0)
    interval = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99, 0.999]
    for num in interval:
        idx = int(len(value_lst) * num)
        value = value_lst[idx]
        print(f"{prefix} {num*100}% {idx}/{len(value_lst)} {value}")
        # print(
        #     f"{prefix} {interval} % value {str()}")
    # exit(0)


# plt.ion()
prev_frames = -1
while True:
    (
        ref_traj_info,
        fbf_traj_info,
        ctrl_res_info,
        contact_force_lst,
        control_force_lst,
        contact_num_lst,
        contact_force_diff_lst,
        control_force_diff_lst,
    ) = load_info("../numeric.log")
    # print(ref_traj_info["q"])
    # exit()
    principle_value_analysis(control_force_diff_lst, "control_force_diff_lst")
    principle_value_analysis(contact_force_diff_lst, "contact_force_diff_lst")
    fbf_err_lst = [
        np.linalg.norm(
            np.array(ref_traj_info["q"][i]) - np.array(fbf_traj_info["q"][i])
        )
        for i in range(len(ref_traj_info["q"]))
    ]
    contact_aware_err_lst = [
        np.linalg.norm(
            np.array(ctrl_res_info["q"][i]) - np.array(fbf_traj_info["q"][i])
        )
        for i in range(len(ref_traj_info["q"]))
    ]
    total_err_lst = [
        np.linalg.norm(
            np.array(ctrl_res_info["q"][i]) - np.array(ref_traj_info["q"][i])
        )
        for i in range(len(ref_traj_info["q"]))
    ]
    # print(np.linalg.norm(fbf_err_lst))
    # print(np.linalg.norm(contact_aware_err_lst))
    # print(np.linalg.norm(total_err_lst))
    # exit(0)
    now_frames = len(control_force_lst)
    # if now_frames != prev_frames:
    if True:
        prev_frames = now_frames

        plt.subplot(3, 5, 1)
        plot_value(ref_traj_info["q"], "ref_q" + suffix, extract_root_y_value)
        plt.subplot(3, 5, 6)
        plot_value(fbf_traj_info["q"], "fbf_q" + suffix, extract_root_y_value)
        plt.subplot(3, 5, 11)
        plot_value(ctrl_res_info["q"], "ctrl_res_q" +
                   suffix, extract_root_y_value)

        plt.subplot(3, 5, 2)
        plot_value(ref_traj_info["qdot"], "ref_qdot" +
                   suffix, extract_root_y_value)
        plt.subplot(3, 5, 7)
        plot_value(fbf_traj_info["qdot"], "fbf_qdot" +
                   suffix, extract_root_y_value)
        plt.subplot(3, 5, 12)
        plot_value(
            ctrl_res_info["qdot"], "ctrl_res_qdot" +
            suffix, extract_root_y_value
        )

        plt.subplot(3, 5, 3)
        plot_value(ref_traj_info["qddot"],
                   "ref_qddot" + suffix, extract_root_y_value)
        plt.subplot(3, 5, 8)
        plot_value(fbf_traj_info["qddot"],
                   "fbf_qddot" + suffix, extract_root_y_value)
        plt.subplot(3, 5, 13)
        plot_value(
            ctrl_res_info["qddot"], "ctrl_res_qddot" +
            suffix, extract_root_y_value
        )
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
        plot_value(
            control_force_lst,
            "ctrl_res_control_force",
            extract_oneleg_revolute_control_force,
        )

        plt.subplot(3, 5, 5)
        plt.cla()
        plt.plot(fbf_err_lst, label="fbf_err")
        plt.plot(contact_aware_err_lst, label="contact_aware_err")
        plt.plot(total_err_lst, label="total_err")
        plt.title("3 control err")

        plt.subplot(3, 5, 10)
        plot_value(contact_force_diff_lst, "contact_force_diff", extract_norm)
        plt.subplot(3, 5, 15)
        plot_value(control_force_diff_lst, "control_force_diff", extract_norm)
        plt.legend()

    # flush
    plt.draw()
    plt.pause(0.1)

    # keep
    # plt.show()
