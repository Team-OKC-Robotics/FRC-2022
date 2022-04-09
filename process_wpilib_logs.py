import matplotlib.pyplot as plt

rpmLog = [[], []]
pidLog = [[], []]
setpointLog = [[], []]
outputLog = [[], []]
intakeSetpointLog = [[], []]
intakeOutputLog = [[], []]
intakePosLog = [[], []]
leftPosLog = [[], []]
leftOutputLog = [[], []]
leftTiltPosLog = [[], []]
leftTiltOutputLog = [[], []]
rightPosLog = [[], []]
rightOutputLog = [[], []]
rightTiltPosLog = [[], []]
rightTiltOutputLog = [[], []]

entries = {
    '"/shooter/rpm"':rpmLog,
    '"shooter/pid-calculate"':pidLog,
    '"/shooter/output"':outputLog,
    '"/shooter/setpoint"':setpointLog,

    '"/intake/setpoint"':intakeSetpointLog,
    '"/intake/output"':intakeOutputLog,
    '"/intake/pos"':intakePosLog,

    '"/climber/leftPos"':leftPosLog,
    '"/climber/leftOutput"':leftOutputLog,
    '"/climber/leftTiltPos"':leftTiltPosLog,
    '"/climber/leftTiltOutput"':leftTiltOutputLog,
    '"/climber/rightPos"':rightPosLog,
    '"/climber/rightOutput"':rightOutputLog,
    '"/climber/rightTiltPos"':rightTiltPosLog,
    '"/climber/rightTiltOutput"':rightTiltOutputLog,
}

# with open(r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\FRC_20220407_215547.csv") as f:
with open(r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\FRC_20220408_210622_OKTU_Q43.csv") as f:
# with open(r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\FRC_20220408_175227.csv") as f:
    log = f.read().split("\n")



for index, line in enumerate(log):
    if line.strip() == "":
        continue
    try:
        timestamp, id, data = line.split(",")
    except Exception:
        print(line)
        continue
    
    try:
        entries[id][0].append(float(timestamp))
        entries[id][1].append(float(data))
    except Exception:
        # print(line)
        continue

plt.figure()
plt.plot(pidLog[0], pidLog[1])
plt.plot(outputLog[0], outputLog[1])
plt.plot(rpmLog[0], rpmLog[1])
plt.plot(setpointLog[0], setpointLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("pid output", "output", "rpm", "setpoint"))

plt.figure()
plt.plot(intakeSetpointLog[0], intakeSetpointLog[1])
plt.plot(intakeOutputLog[0], intakeOutputLog[1])
plt.plot(intakePosLog[0], intakePosLog[1])
plt.legend(("intake setpoint", "intake output", "intake pos"))

plt.figure()
plt.plot(leftOutputLog[0], leftOutputLog[1])
plt.plot(leftPosLog[0], leftPosLog[1])
plt.plot(leftTiltOutputLog[0], leftTiltOutputLog[1])
plt.plot(leftTiltPosLog[0], leftTiltPosLog[1])

plt.plot(rightOutputLog[0], rightOutputLog[1])
plt.plot(rightPosLog[0], rightPosLog[1])
plt.plot(rightTiltOutputLog[0], rightTiltOutputLog[1])
plt.plot(rightTiltPosLog[0], rightTiltPosLog[1])
plt.legend(("left output", "left pos", "left tilt output", "left tilt pos", "right output", "right pos", "right tilt output", "right tilt pos"))

plt.show()

