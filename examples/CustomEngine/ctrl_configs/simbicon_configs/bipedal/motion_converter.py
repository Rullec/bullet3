import json
import numpy as np


def euler_to_quaternion(euler, axis=[1, 0, 0]):
    assert(type(euler) is float)
    assert(len(axis) is 3)
    theta_2 = euler / 2
    return [np.cos(theta_2),
            axis[0] * np.sin(theta_2),
            axis[1] * np.sin(theta_2),
            axis[2] * np.sin(theta_2),
            ]


def pose_converter(raw_pose):
    print(f"raw pose {raw_pose} {np.size(raw_pose)}")
    assert(np.size(raw_pose) is 9)
    root = raw_pose[0:3]
    left_leg = raw_pose[3:6]
    right_leg = raw_pose[6:9]
    print(f"\t root {root}")
    print(f"\t left_leg {left_leg}")
    print(f"\t right_leg {right_leg}")
    new_pose = root + euler_to_quaternion(
        left_leg[0]) + left_leg[1:] + euler_to_quaternion(right_leg[0]) + right_leg[1:]
    print(
        f"\t new pose {new_pose}")
    return new_pose


raw_path = "target_bipedal_sph_legs_jog.json"

with open(raw_path, 'r') as f:
    cont = json.load(f)
    # print(cont)
    for item in cont["list"]:
        pose = item["char_pose"]
        item["char_pose"] = pose_converter(pose)
    print(f"new cont  {cont}")

with open(raw_path, 'w') as f:
    json.dump(cont, f)
