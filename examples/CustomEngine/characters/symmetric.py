import json
path = "ankle_universal_hip_ball.json"


def check_the_same(v1, v2):
    all_keys = v1.keys()
    name1 = v1["Name"]
    name2 = v2["Name"]
    for cur_key in all_keys:
        if (cur_key != "Name") and (cur_key != "ID") and (cur_key != "ParentJoint") and (cur_key != "Parent"):
            assert v2[cur_key] == v1[cur_key], f"key {cur_key}: {name1} {v1[cur_key]} != {name2} {v2[cur_key]}"
    return


def check_symmetric(name_value_lst):
    for cur_name in name_value_lst:
        this_value = name_value_lst[cur_name]

        another_name = ""
        if cur_name.find("Left") != -1:
            another_name = "Right" + cur_name[4:]
        elif cur_name.find("Right") != -1:
            another_name = "Left" + cur_name[5:]
        else:
            print(f"name {cur_name} ignored")
            continue
        another_value = name_value_lst[another_name]
        check_the_same(this_value, another_value)


with open(path, 'r') as f:
    value = json.load(f)
    for i in value.keys():
        print(f"--------begin to check {i}-------")
        cur_v = value[i]
        name_value_lst = {}

        if type(cur_v) == dict:
            cur_v = cur_v["Joints"]
        for comp in cur_v:
            name = comp["Name"]
            name_value_lst[name] = comp

        check_symmetric(name_value_lst)
