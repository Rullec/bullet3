import os
import re
import matplotlib.pyplot as plt


def get_q_diff_norm(filepath):
    with open(filepath, 'r') as f:
        cont = f.readlines()
    q_diff_lst = []
    for line in cont:
        line = line.strip()
        if -1 != line.find("norm"):
            splited = re.split("\s|,", line)
            print(splited)
            q_diff_lst.append(float(splited[4]))
    return q_diff_lst


q_diff = get_q_diff_norm("log")
plt.plot(q_diff)
plt.title("q diff norm curve")
plt.show()
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

# plt.subplot(1, 3, 1)
# plt.plot(q_new, label="q")
# plt.legend()
# plt.title("Fig q diff norm")

# plt.subplot(1, 3, 2)
# plt.plot(qdot_new, label="qdot")
# plt.legend()
# plt.title("Fig qdot diff norm")

# plt.subplot(1, 3, 3)
# plt.plot(qddot_new, label="qddot")
# plt.legend()
# plt.title("Fig qddot diff norm")
# plt.show()
