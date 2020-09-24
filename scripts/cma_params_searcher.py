from xvfbwrapper import Xvfb
import numpy as np
import cma
import os
import time
PARAM_NAME = ["dynamic_pos_energy_coef"]


def GetParamterSize():
    return len(PARAM_NAME)


class Simulator:
    def __init__(self, data):
        self.data = data
        return

    def write_down_data(self):
        
    def run(self):
        a = time.time()
        cmd = "cd .. ; ./App_ExampleBrowser"
        print(f"begin to run {cmd}")
        write_down_data()
        output = os.popen(cmd).readlines()
        q_diff_lst = [float(line.split()[6]) / 1e-3 for line in output if -
                      1 != line.find("[debug] ctrl_res and ref_traj q diff")]
        # [float(line.split()[6])/1e-3 for line in output if line is not None]
        b = time.time()
        reward = sum(q_diff_lst)
        print(f"cost {b - a} s, reward {reward}")
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
vdisplay = Xvfb()
vdisplay.start()
sim = Simulator(None)
sim.run()
vdisplay.stop()
# def f(data):
#     sim = Simulator(data)
#     return sim.run()


# if __name__ == "__main__":
#     es = cma.CMAEvolutionStrategy(np.random.rand(15), 1)
#     while not es.stop():
#         X = es.ask()
#         # print(X)

#         es.tell(X, [f(x) for x in X])
#         es.disp()  # doctest: +ELLIPSIS
#     es.result_pretty()
