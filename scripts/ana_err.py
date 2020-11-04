import os
import re
import matplotlib.pyplot as plt


def get_q_diff(line):
    return float(line.split()[3][:-1])


def get_qdot_diff(line):
    return float(line.split()[6])


def get_ctrl_diff(log_path, get_method):

    ctrl_and_ref_diff_lst = []

    with open(log_path) as f:
        for line in f.readlines():
            if line.find("[nqr]") != -1:
                if line.find("diff") != -1:
                    line = line.strip()
                    q_diff = get_method(line)
                    ctrl_and_ref_diff_lst.append(q_diff)

    return ctrl_and_ref_diff_lst


ctrl_and_ref_q_diff_lst = get_ctrl_diff(
    "./log", get_q_diff)
ctrl_and_ref_qdot_diff_lst = get_ctrl_diff(
    "./log", get_qdot_diff)


def draw(idx, item, ctrl_and_ref_diff_lst):
    plt.subplot(1, 3, idx)
    plt.plot(ctrl_and_ref_diff_lst,
             label=f"control result - ref traj")

    plt.title(f"{item} diff norm")
    plt.legend()


# plt.title("show q diff norm in 3 feature vectors")
draw(1, "q", ctrl_and_ref_q_diff_lst)
draw(2, "qdot", ctrl_and_ref_qdot_diff_lst)

plt.show()
# plt.subplot(1, 3, 2)
# plt.plot(pos_qdot, label="pos_ctrled")
# plt.plot(vel_qdot, label="vel_ctrled")
# plt.plot(accel_qdot, label="accel_ctrled")
# plt.legend()
# plt.title("show qdot diff norm in 3 feature vectors")

# plt.subplot(1, 3, 3)
# plt.plot(pos_qddot, label="pos_ctrled")
# plt.plot(vel_qddot, label="vel_ctrled")
# plt.plot(accel_qddot, label="accel_ctrled")
# plt.legend()
# plt.title("show qddot diff norm in 3 feature vectors")

# plt.show()
