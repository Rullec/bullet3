import sys

if __name__ == "__main__":
    logfile = sys.argv[1]
    total_time = 0
    cnt = 0
    with open(logfile) as f:
        for line in f.readlines():
            if line.find("stepsim cost time") != -1:
                cost = float(line.split()[-2])
                total_time += cost
                cnt += 1
    print(logfile)
    print(f"avg cost time {total_time/cnt} ms")
    print(f"total cost time {total_time} ms")
