import matplotlib.pyplot as plt
from math import isclose, inf, nan
import copy, random

rpmLog = [[], []]
pidLog = [[], []]
setpointLog = [[], []]
outputLog = [[], []]
hasBallLog = [[], []]

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

xDifferenceLog = [[], []]

rightDriveOutputLog = [[], []]
leftDriveOutputLog = [[], []]
headingLog = [[], []]
rightDriveVelocityLog = [[], []]
leftDriveVelocityLog = [[], []]

entries = {
    '"/shooter/rpm"':rpmLog,
    '"shooter/pid-calculate"':pidLog,
    '"/shooter/output"':outputLog,
    '"/shooter/setpoint"':setpointLog,
    '"/shooter/hasBall"':hasBallLog,

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

    '"/vision/xdifference"':xDifferenceLog,

    '"/drivetrain/rightOutputLog"':rightDriveOutputLog,
    '"/drivetrain/leftOutputLog"':leftDriveOutputLog,
    '"/drivetrain/rightVelocity"':rightDriveVelocityLog,
    '"/drivetrain/leftVelocity"':leftDriveVelocityLog,
    '"/drivetrain/heading"':headingLog,
}

# log = r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\FRC_20220408_210622_OKTU_Q43.csv"
log = r"C:\Users\teamo\Documents\GitHub\FRC-2022\logs\all\drivetrain\FRC_20220418_191842.csv"
with open(log) as f:
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
    except KeyError:
        continue
    except Exception:
        entries[id][1].append(1000 if bool(data) else 0)
        # print(line)
        continue

plt.figure()
plt.plot(pidLog[0], pidLog[1])
plt.plot(outputLog[0], outputLog[1])
plt.plot(rpmLog[0], rpmLog[1])
plt.plot(setpointLog[0], setpointLog[1])
plt.plot(hasBallLog[0], hasBallLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("pid output", "output", "rpm", "setpoint", "has ball"))

plt.figure()
plt.plot(intakeSetpointLog[0], intakeSetpointLog[1])
plt.plot(intakeOutputLog[0], intakeOutputLog[1])
plt.plot(intakePosLog[0], intakePosLog[1])
plt.legend(("intake setpoint", "intake output", "intake pos"))

# plt.figure()
# plt.plot(leftOutputLog[0], leftOutputLog[1])
# plt.plot(leftPosLog[0], leftPosLog[1])
# plt.plot(leftTiltOutputLog[0], leftTiltOutputLog[1])
# plt.plot(leftTiltPosLog[0], leftTiltPosLog[1])

# plt.plot(rightOutputLog[0], rightOutputLog[1])
# plt.plot(rightPosLog[0], rightPosLog[1])
# plt.plot(rightTiltOutputLog[0], rightTiltOutputLog[1])
# plt.plot(rightTiltPosLog[0], rightTiltPosLog[1])
# plt.legend(("left output", "left pos", "left tilt output", "left tilt pos", "right output", "right pos", "right tilt output", "right tilt pos"))

plt.figure()
plt.plot(xDifferenceLog[0], xDifferenceLog[1])
plt.plot(leftDriveOutputLog[0], leftDriveOutputLog[1])
plt.legend(("vision error", "left drivetrain output"))

plt.figure()
plt.plot(rightDriveOutputLog[0], rightDriveOutputLog[1])
plt.plot(leftDriveOutputLog[0], leftDriveOutputLog[1])
plt.plot(rightDriveVelocityLog[0], rightDriveVelocityLog[1])
plt.plot(leftDriveVelocityLog[0], leftDriveVelocityLog[1])
plt.plot(headingLog[0], headingLog[1])
plt.legend(("right drivetrain output", "left drivetrain output", "right velocity", "left velocity", "heading"))

plt.show()
raise SystemExit

def calculate(state, output, param):
    try:
        return param[0] * state + param[1] * output
    except Exception:
        print(state)
        print(output)
        raise SystemExit

def calculate_error(actualRPM, predictedRPM):
    error = 0
    for x in range(len(actualRPM)):
        error += abs(actualRPM[x] - predictedRPM[x])
    # error /= len(actualRPM)
    return error

# go through the output log and try to find an output close to the timestamp
# if the timestamp is not close to any output ones, then return 0
# finds an output close to the 
def find(timestamp, last_index):
    global outputLog

    for index, outputTime in enumerate(outputLog[0][max(0, last_index-3):]): # for every timestamp in the output list
        # if outputTime - timestamp < 0.03: # if the timestamp is close to the output stamp
        #     return outputLog[1][index] # return that output
        if isclose(outputTime, timestamp, abs_tol=0.03):
            return outputLog[1][index], index # return that output
        elif outputTime - timestamp > 0.8: # otherwise if it gets too large
            return 0, index # then the output is 0
    return 0, index

# if the timestamp is higher and the output time is early, then result of sub is positive
# it the output time is later and the timestamp is lower, then result of sub is also positive?

predictedRPMLog = [[], []]
previousState = 0
state = 0
last_idx = 0
param = [0.96, 750]
# 0.4 * B = 9000
for index, rpm in enumerate(rpmLog[1]):
    output, last_idx = find(rpmLog[0][index], last_idx)
    state = calculate(previousState, output, param)

    predictedRPMLog[0].append(rpmLog[0][index]) # timestamp
    predictedRPMLog[1].append(state) # append the predicted RPM at that timestamp

    previousState = state

error = abs(calculate_error(rpmLog[1], predictedRPMLog[1]))


print(calculate_error(rpmLog[1], predictedRPMLog[1]))

# plt.plot(predictedRPMLog[0], predictedRPMLog[1])
# plt.show()

# raise SystemExit

pop = []
pop_size = 50
for x in range(pop_size):
    # pop.append([abs(random.random()), abs(random.random() * 100), 0]) # A, B, fitness
    pop.append([0.9, 2000, 0]) # A, B, fitness

best_error = 10000000
generations = 0
try:
    while best_error > 1000:
    # while generations < 5: # temporary only run for 1 generation
        for param in pop:
            predictedRPMLog = [[], []]
            previousState = 0
            state = 0
            last_idx = 0

            for index, rpm in enumerate(rpmLog[1]):
                output, last_idx = find(rpmLog[0][index], last_idx)
                state = calculate(previousState, output, param)

                predictedRPMLog[0].append(rpmLog[0][index]) # timestamp

                predictedRPMLog[1].append(state) # append the predicted RPM at that timestamp

                previousState = state
            
            error = abs(calculate_error(rpmLog[1], predictedRPMLog[1]))
            param[2] = error
            # if error == inf or error == nan:
            #     pass
            # else:
            #     # print(error)
            

            if (error < best_error):
                best_error = error
        print(best_error)
        
        pop.sort(key=lambda x:x[2], reverse=False) # sort according to lower fitness
        # print(pop)
        pop = pop[:int(len(pop)/2)] # keep the top half
        pop += copy.deepcopy(pop) # rebuild using a copy of the same half

        
        for param in pop[1:]: # don't mutate the best one
            # if random.random() < 0.7:
            param[0] = param[0] + param[0] * random.uniform(-10, 10)

            # if random.random() < 0.7:
            param[1] += param[1] + param[1] * random.uniform(-10, 10)

        generations += 1
except KeyboardInterrupt:
    pass
print(generations)
print(pop[0])


plt.plot(predictedRPMLog[0], predictedRPMLog[1])
plt.show()

# x* = Ax + Bu
# def simulate_system(params):
#     global outputLog, rpmLog

#     for i in range(len(rpmLog)):
#         state = params[0] * state + params[1] * control

"""
before "last index" optimization
   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
    298/1    0.001    0.000 2145.119 2145.119 {built-in method builtins.exec}
        1    0.440    0.440 2145.119 2145.119 process_wpilib_logs.py:1(<module>)
        1    0.000    0.000 2113.381 2113.381 pyplot.py:323(show)
        1    0.000    0.000 2113.381 2113.381 backend_bases.py:3510(show)
        1    0.000    0.000 2113.368 2113.368 _backend_tk.py:948(mainloop)
        1    0.000    0.000 2113.368 2113.368 __init__.py:1456(mainloop)
        1 2113.160 2113.160 2113.368 2113.368 {method 'mainloop' of '_tkinter.tkapp' objects}
   334890   19.374    0.000   29.810    0.000 process_wpilib_logs.py:112(find)
151073880   10.436    0.000   10.436    0.000 {built-in method math.isclose}


after
   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
    298/1    0.001    0.000   23.008   23.008 {built-in method builtins.exec}
        1    0.447    0.447   23.008   23.008 process_wpilib_logs.py:1(<module>)
   334890   10.758    0.000   16.301    0.000 process_wpilib_logs.py:112(find)
 76211760    5.500    0.000    5.500    0.000 {built-in method math.isclose}
        1    0.000    0.000    4.753    4.753 pyplot.py:323(show)
"""