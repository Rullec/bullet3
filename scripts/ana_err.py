import os
import re
import matplotlib.pyplot as plt


def get_q_diff(line):
    return float(line.split()[6])


def get_qdot_diff(line):
    return float(line.split()[9])


def get_qddot_diff(line):
    return float(line.split()[13])


def get_ctrl_diff(log_path, get_method):

    ctrl_and_ref_diff_lst, ctrl_and_target_diff_lst, ref_and_target_diff_lst = [], [], []

    with open(log_path) as f:
        for line in f.readlines():
            if line.find("diff") != -1:
                line = line.strip()
                q_diff = get_method(line)
                if line.find("ctrl_res") == -1:
                    # handle ref traj and target diff
                    ref_and_target_diff_lst.append(q_diff)
                    # ref_and_target_qdot_diff_lst.append(qdot_diff)
                    # ref_and_target_qddot_diff_lst.append(qddot_diff)
                else:
                    if line.find("ref_traj") != -1:
                        # ref_traj and ctrl_res
                        ctrl_and_ref_diff_lst.append(q_diff)
                    elif line.find("target_traj") != -1:
                        ctrl_and_target_diff_lst.append(q_diff)
                    else:
                        raise ValueError(f"fail to handle {line}")

    return ctrl_and_ref_diff_lst, ctrl_and_target_diff_lst, ref_and_target_diff_lst


ctrl_and_ref_q_diff_lst, ctrl_and_target_q_diff_lst, ref_and_target_q_diff_lst = get_ctrl_diff(
    "./log", get_q_diff)
ctrl_and_ref_qdot_diff_lst, ctrl_and_target_qdot_diff_lst, ref_and_target_qdot_diff_lst = get_ctrl_diff(
    "./log", get_qdot_diff)
ctrl_and_ref_qddot_diff_lst, ctrl_and_target_qddot_diff_lst, ref_and_target_qddot_diff_lst = get_ctrl_diff(
    "./log", get_qddot_diff)


def draw(idx, item, ctrl_and_ref_diff_lst, ctrl_and_target_diff_lst, ref_and_target_diff_lst):
    plt.subplot(1, 3, idx)
    plt.plot(ctrl_and_ref_diff_lst,
             label=f"control result - ref traj")
    plt.plot(ctrl_and_target_diff_lst,
             label=f"control result - target traj")
    plt.plot(ref_and_target_diff_lst,
             label=f"ref traj - target traj")
    plt.title(f"{item} diff norm")
    plt.legend()


# plt.title("show q diff norm in 3 feature vectors")
draw(1, "q", ctrl_and_ref_q_diff_lst,
     ctrl_and_target_q_diff_lst, ref_and_target_q_diff_lst)
draw(2, "qdot", ctrl_and_ref_qdot_diff_lst,
     ctrl_and_target_qdot_diff_lst, ref_and_target_qdot_diff_lst)
draw(3, "qddot", ctrl_and_ref_qddot_diff_lst,
     ctrl_and_target_qddot_diff_lst, ref_and_target_qddot_diff_lst)
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
