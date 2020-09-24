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
    "control_force_close_to_origin_coef",
    "contact_force_close_to_origin_coef",
    "end_effector_pos_coef",
    "end_effector_vel_coef",
    "end_effector_orient_coef",
    "root_pos_coef",
    "root_vel_coef",
    "root_orientation_coef"
]


def GetParamterSize():
    return len(PARAM_NAME)


class Simulator:
    def __init__(self, data):
        assert data is not None
        self.data = data
        assert(len(self.data) == GetParamterSize())
        return

    def write_down_data(self):
        path = "/home/xudong/Projects/bullet3/examples/CustomEngine/contact_aware_adviser_config.json"
        with open(path, 'r') as f:
            root = json.load(f)
        for i in range(GetParamterSize()):
            key = PARAM_NAME[i]
            value = self.data[i]
            root["ctrl_config"][key] = value
        with open(path, 'w') as f:
            json.dump(root, f, indent=True)

    def run(self):
        a = time.time()
        cmd = "cd .. ; ./App_ExampleBrowser"
        # print(f"begin to run {cmd}")
        output = os.popen(cmd).readlines()
        q_diff_lst = [float(line.split()[6]) for line in output if -
                      1 != line.find("[debug] ctrl_res and ref_traj q diff")]
        # [float(line.split()[6])/1e-3 for line in output if line is not None]
        b = time.time()
        reward = sum(q_diff_lst)
        # print(f"[log] cost {b - a} s, reward {reward}")
        return reward
        # filename = "tmp"
        # res = self.data[0]
        # with open(filename, 'w') as f:
        #     f.write(str(self.data[0]))

        # time.sleep(1)
        # with open(filename, 'r') as f:
        #     new_res = float(f.readlines()[0].split()[0])
        #     assert(new_res == res)

        # return np.linalg.norm(np.array(self.data))

# sim = Simulator([1] * GetParamterSize())
# sim.run()
# sim.write_down_data()


# def f(data):

#     if max(data) > 10 or min(data) < 1e-3:
#         return 1e10
#     sim = Simulator(data)
#     return sim.run()

# def add_lock(file):

FILE = "counter.txt"


def f(data):
    file = open(FILE, 'a+')
    sim = Simulator(data)
    fcntl.flock(file.fileno(), fcntl.LOCK_EX)
    # begin to protect
    # res = np.linalg.norm(data)
    # with open('tmp', 'w') as f:
    #     f.write(str(res))

    # with open("tmp", 'r') as f:
    #     new_res = float(f.readlines()[0].split()[0])
    # print(f"res {res} new res {new_res}")
    # assert res == new_res
    time.sleep(0.2)
    sim.write_down_data()
    # end to protect
    fcntl.flock(file, fcntl.LOCK_UN)
    res = sim.run()
    return res


vdisplay = Xvfb()
vdisplay.start()
pool = Pool(14)
if __name__ == "__main__":
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
vdisplay.stop()
