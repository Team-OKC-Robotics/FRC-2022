import matplotlib.pyplot as plt

rpmLog = [[], []]
pidLog = [[], []]
setpointLog = [[], []]
outputLog = [[], []]

with open(r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\FRC_20220404_233808.csv") as f:
    log = f.read().split("\n")

for index, line in enumerate(log):
    if line.strip() == "":
        continue
    try:
        timestamp, id, data = line.split(",")
    except Exception:
        print(line)
        continue
    # print(timestamp, id, data)
    if (id == '"/shooter/rpm"'):
        rpmLog[0].append(float(timestamp))
        rpmLog[1].append(float(data))
    elif id == '"/shooter/pid-calculate"':
        pidLog[0].append(float(timestamp))
        pidLog[1].append(float(data))
    elif id == '"/shooter/output"':
        outputLog[0].append(float(timestamp))
        outputLog[1].append(float(data) * 4000) # so when plotted changes can be seen. this is not the right way to handle this
        # I don't care
    elif id == '"/shooter/setpoint"':
        setpointLog[0].append(float(timestamp))
        setpointLog[1].append(float(data))

# print(len(rpmLog)) # 1603
# print(len(outputLog)) # 634
# print(len(setpointLog)) # 634
# print(len(pidLog)) # 634


# plt.figure()
# plt.title("just a title")
# plt.plot(outputLog[0], outputLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("output (% max)")

# plt.figure()
# plt.plot(rpmLog[0], rpmLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("RPM (RPM per 200 ms or something)")

# plt.figure()
# plt.plot(setpointLog[0], setpointLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("setpoint (RPM-ish)")

# plt.figure()
# plt.plot(pidLog[0], pidLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("pid output (calculated)")
# plt.show()

plt.figure()
plt.plot(pidLog[0], pidLog[1])
plt.plot(outputLog[0], outputLog[1])
plt.plot(rpmLog[0], rpmLog[1])
plt.plot(setpointLog[0], setpointLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("pid output", "output", "rpm", "setpoint"))
plt.show()
