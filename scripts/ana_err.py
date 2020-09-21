import os
import re
import matplotlib.pyplot as plt


def get_q_qdot_qddot_error(filepath):
    q_lst = []
    qdot_lst = []
    qddot_lst = []
    with open(filepath) as f:
        cont = f.readlines()
        for line in cont:
            if -1 != line.find("true"):
                line = line.strip()
                res = re.split('\s|,', line)
                print(res)
                qddot_lst.append(float(res[4]))
                qdot_lst.append(float(res[9]))
                q_lst.append(float(res[14]))
    return q_lst, qdot_lst, qddot_lst


pos_q, pos_qdot, pos_qddot = get_q_qdot_qddot_error("pos.log")
vel_q, vel_qdot, vel_qddot = get_q_qdot_qddot_error("vel.log")
accel_q, accel_qdot, accel_qddot = get_q_qdot_qddot_error("accel.log")


# plt.plot(q_diff)
# plt.title("q diff norm curve")
# plt.show()

# get_q_diff_norm("log")
# def get_q_and_qdot_diff(filepath):
#     with open(filepath, 'r') as f:
#         cont = f.readlines()
#     q_diff_lst, qdot_diff_lst, qddot_diff_lst = [], [], []
#     for line in cont:
#         line = line.strip()
#         if -1 != line.find("[true]"):
#             splited = re.split("\s|,", line)
#             qddot_diff_lst.append(float(splited[4]))
#             qdot_diff_lst.append(float(splited[9]))
#             q_diff_lst.append(float(splited[14]))
#             # print(line)
#     return q_diff_lst, qdot_diff_lst, qddot_diff_lst


# q_new, qdot_new, qddot_new = get_q_and_qdot_diff("new.log")
# q_old, qdot_old, qddot_old = get_q_and_qdot_diff("old.log")

plt.subplot(1, 3, 1)
plt.plot(pos_q, label="pos_ctrled")
plt.plot(vel_q, label="vel_ctrled")
plt.plot(accel_q, label="accel_ctrled")
plt.legend()
plt.title("show q diff norm in 3 feature vectors")

plt.subplot(1, 3, 2)
plt.plot(pos_qdot, label="pos_ctrled")
plt.plot(vel_qdot, label="vel_ctrled")
plt.plot(accel_qdot, label="accel_ctrled")
plt.legend()
plt.title("show qdot diff norm in 3 feature vectors")

plt.subplot(1, 3, 3)
plt.plot(pos_qddot, label="pos_ctrled")
plt.plot(vel_qddot, label="vel_ctrled")
plt.plot(accel_qddot, label="accel_ctrled")
plt.legend()
plt.title("show qddot diff norm in 3 feature vectors")

plt.show()
