import fcntl
from multiprocessing import Pool
from xvfbwrapper import Xvfb
import numpy as np
import cma
import os
import time
import json
PARAM_NAME = [
    "dynamic_pos_energy_coef",
    "dynamic_vel_energy_coef",
    "dynamic_accel_energy_coef",
    "control_force_coef",
    "contact_force_coef",
    "end_effector_pos_coef",
    "end_effector_vel_coef",
    "end_effector_orient_coef",
    "root_pos_coef",
    "root_vel_coef",
    "root_orientation_coef"
]

trajs = [
    "/home/xudong/Projects/DeepMimic/data/id_test/sample_fullbody/traj_fullbody_137757747.json",
    "/home/xudong/Projects/DeepMimic/data/id_test/sample_fullbody/traj_fullbody_143582365.json",
    "/home/xudong/Projects/DeepMimic/data/id_test/sample_fullbody/traj_fullbody_1974042356.json",
    "/home/xudong/Projects/DeepMimic/data/id_test/sample_fullbody/traj_fullbody_606226380.json",
    "/home/xudong/Projects/DeepMimic/data/id_test/sample_fullbody/traj_fullbody_847986793.json"
]


def GetParamterSize():
    return len(PARAM_NAME)


class Simulator:
    def __init__(self, data):
        assert data is not None
        self.data = data
        assert(len(self.data) == GetParamterSize())
        return

    def get_the_command(self, traj_path):
        if os.path.exists(traj_path) == False:
            raise ValueError(f"traj path {traj_path} doesn't exist")
        cmd_base = f"cd .. ; ./App_CustomEngine_noGUI traj {traj_path} "
        assert len(self.data) == GetParamterSize()

        for idx, name in enumerate(PARAM_NAME):
            cmd_single = f" coef_{name} {self.data[idx]} "
            cmd_base += cmd_single
        return cmd_base

    def calculate_reward(self, output):
        q_diff_lst = [float(line.split()[6]) for line in output if -
                      1 != line.find("[debug] ctrl_res and ref_traj q diff")]

        for line in output:
            if -1 != line.find("the cartesian velocity of model has exploded"):
                print("[exp] cartesian vel exploded")
                q_diff_lst.append(1e6)
        print(f"[log] running num of frame: {len(q_diff_lst)}")
        reward = sum(q_diff_lst) / len(q_diff_lst)
        return reward

    def run_single_traj(self, traj_path):
        cmd = self.get_the_command(traj_path)
        # print(f"cmd is: {cmd}")
        # exit(0)
        output = os.popen(cmd).readlines()
        single_reward = self.calculate_reward(output)
        return single_reward

    def run(self):
        total_reward = 0
        for idx,  traj in enumerate(trajs):
            st = time.time()
            traj_reward = self.run_single_traj(traj)
            ed = time.time()
            total_reward += traj_reward
            print(f"[log] traj {idx} cost {ed - st} s, reward {traj_reward}")
        total_reward /= len(trajs)
        print(f"[log] trajs final reard {total_reward}")
        return total_reward


def f(data):
    sim = Simulator(data)
    res = sim.run()
    return res


vdisplay = Xvfb()
vdisplay.start()
pool = Pool(14)
if __name__ == "__main__":
    # training
    options = cma.CMAOptions()
    options['popsize'] = 14
    options['bounds'] = [[1e-5] * GetParamterSize(), [10] * GetParamterSize()]
    es = cma.CMAEvolutionStrategy(
        [1] * GetParamterSize(), 0.5, options)
    while not es.stop():
        X = es.ask()
        with open("solutions.txt", 'a+') as fp:
            fp.write("--------------------------\n")
            for i in X:
                fp.write(str(i) + '\n')
        es.tell(X, pool.map(f, X))
        es.disp()
    es.result_pretty()
    # data = [1] * GetParamterSize()
    # f(data)
vdisplay.stop()
