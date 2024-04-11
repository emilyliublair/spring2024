import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue
import random
import copy

# global vars
time_to_move = 3
counter_threshold = 3
rand_t_low = 4
rand_t_high = 7
rand_amp_low = 7
rand_amp_high = 7
amt_to_move = .1 # 110 / 45
followBuffer = 2

# user var
coordinate = [None, None]


#arm 1
queueOne = Queue()
initPosPoppingOne = [0.0, -42.0, 5.0, 71.2, -10.1, 58.4, 35.2]
# initPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
finPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
leftPopping = [0, 15.6, -25, 142.9, -10.1, 10.1, 35.2]
rightPopping = [0, 15.6, 65, 142.9, -10.1, 10.1, 35.2]
currPosOne = initPosPoppingOne
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
prevStatusOne = "back"
counterOne = 0
statusOne = False
startTrackingOne = False

queue = [queueOne]
initPosPopping = [initPosPoppingOne]
finPosPopping = [finPosPoppingOne]
currPos = [currPosOne]
currVel = [currVelOne]
currTime = [currTimeOne]
prevStatus = [prevStatusOne]
counter = [counterOne]
status = [statusOne]
startTracking = [startTrackingOne]

no_one_tracker = True
z1_2_slope = -0.57
z1_2_intercept = 331
overlap = 20
def zone_2(x, y):
    line_y = z1_2_slope * x + (z1_2_intercept - overlap)
    inzone = x < 300 and y > line_y
    return inzone

def headmvmt(x):
    # return .378*x-55.77
    return .381*x - 49.384

def fifth_poly(q_i, q_f, v_i, vf, t):
    if (t == 0):
        print("STOP")
        print(q_i)
        print(q_f)
        print(v_i)
        print(vf)
    # time/0.005
    traj_t = np.arange(0, t, 0.004)
    dq_i = v_i
    dq_f = vf
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * t ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * t - (3 * ddq_f - ddq_i) * t ** 2)
    a4 = 1 / (2 * t ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * t + (3 * ddq_f - 2 * ddq_i) * t ** 2)
    a5 = 1 / (2 * t ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * t - (ddq_f - ddq_i) * t ** 2)
    traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
    traj_vel = a1  + 2*a2 * traj_t ** 1 + 3*a3 * traj_t ** 2 + 4*a4 * traj_t ** 3 + 5*a5 * traj_t ** 4
    return traj_pos, traj_vel

def setup(strumD):
    i = 0
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring[i].copy()
        i += 1
        # angle[0] = angle[0]- strumD/2
        print(angle)
        a.set_servo_angle(angle=angle, wait=True, speed=10, acceleration=0.25, is_radian=False)

def pose_to_pose(initPos, finPos, vi, vf, t, armNum):
    print(armNum)
    global queue
    with queue[armNum].mutex:
        queue[armNum].queue.clear()
        queue[armNum].all_tasks_done.notify_all()
        queue[armNum].unfinished_tasks = 0

    currArmAngles = [round(arms[armNum].angles[0], 1), round(arms[armNum].angles[1], 1), round(arms[armNum].angles[2], 1),
                     round(arms[armNum].angles[3], 1), round(arms[armNum].angles[4], 1), round(arms[armNum].angles[5], 1),
                     round(arms[armNum].angles[6], 1)]

    if t == 0:
        print("T IS ZERO")
        print(arms[armNum].angles)
        print(finPos)
        t = t/0
        # arms[armNum].set_servo_angle(angle=finPos, wait=True, speed=10, acceleration=0.25, is_radian=False)
        # currPos[armNum] = finPos
    else:
        posTrajs = []
        velTrajs = []
        for i in range(len(initPos)):  # generate trajs and vels for each joint
            tempTraj, tempVel = fifth_poly(initPos[i], finPos[i], vi[i], vf, t)  # tempTraj, tempVel has t/.004 length
            posTrajs.append(tempTraj)
            velTrajs.append(tempVel)
        reformatPos = [currArmAngles]
        reformatVel = [currVel[armNum]]
        for i in range(len(posTrajs[0])):
            posRow = []
            velRow = []
            for j in range(7):
                posRow.append(posTrajs[j][i])
                velRow.append(velTrajs[j][i])
            if finPos == finPosPopping[armNum]:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] + ((i+1) * 0.004), reformatVel[-1],
                          velRow])  # each queue item has curr_pos, next_pos, next_time, curr_vel, next_vel for all joints
            else:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] - ((i + 1) * 0.004), reformatVel[-1],
                                   velRow])
            reformatPos.append(posRow)
            reformatVel.append(velRow)

def visionThread():
    global no_one_tracker
    global arms
    global counter
    global prevStatus
    global status
    global coordinate

    while True:
        data, addr = sock.recvfrom(1024)

        if ("noone" in data.decode("utf-8")):
            coordinate = [None, None]
            for i in range(len(arms)):
                if prevStatus[i] == "back":
                    counter[i] += 1
                else:
                    counter[i] = 0
                    prevStatus[i] = "back"
                if counter[i] > 5 and status[i]:
                    status[i] = False
                    print("arm" + str(i) + ") go back, no one detected")
                    pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)
        else:
            rec_arr = ast.literal_eval(data.decode("utf-8"))
            armDetect = [False]
            for i in range(len(rec_arr)):
                coordinates = rec_arr[i].split("/")[0].split(",")
                vel = rec_arr[i].split("/")[1].split(",")
                x_coord = float(coordinates[0])
                y_coord = float(coordinates[1])
                coordinate = [x_coord, y_coord]
                x_vel = float(vel[0])
                y_vel = float(vel[1])
                if zone_2(x_coord, y_coord):
                    armDetect[0] = True
                    if prevStatus[0] == "out":
                        counter[0] += 1
                    else:
                        counter[0] = 0
                        prevStatus[0] = "out"
                    if counter[0] > counter_threshold and not(status[0]):
                        status[0] = True
                        print("arm1) out")
                        pose_to_pose(currPos[0], finPosPopping[0], currVel[0], 0, time_to_move - currTime[0], 0)

            for i in range(len(armDetect)):
                if not (armDetect[i]):
                    if prevStatus[i] == "back":
                        counter[i] += 1
                    else:
                        counter[i] = 0
                        prevStatus[i] = "back"
                    if counter[i] > counter_threshold and status[i]:
                        status[i] = False
                        print("arm" + str(i) + ") go back, no one detected IN RANGE")
                        pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)

def robotThread():
    global coordinate
    # next_pos = arms[0].angles
    while True:
        start_time = time.time()
        tts = time.time() - start_time
        for i in range(len(arms)):
            if not(queue[i].empty()):
                startTracking[i] = False
                info = queue[i].get()
                next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1),
                            round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

                arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
                currPos[i] = next_pos
                currTime[i] = info[2]
                currVel[i] = info[4]
            else: # save value and go to target goal
                # currentp = arms[i].angles


                if (status[i]):
                    if startTracking[i]:
                        next_pos1 = currPos[i]
                    else:
                        next_pos1 = arms[i].angles
                        startTracking[i] = True
                    thirdJVel = 0.0
                    if coordinate[0] is not None and zone_2(coordinate[0], coordinate[1]):
                        head = headmvmt(coordinate[0])
                        if head < -25: head = -25
                        if head > 65: head = 65
                        if (head > currPos[i][2] + followBuffer):
                            next_pos1[2] = currPos[i][2] + amt_to_move
                            thirdJVel = amt_to_move
                        elif head < currPos[i][2] - followBuffer:
                            next_pos1[2] = currPos[i][2] - amt_to_move
                            thirdJVel = -amt_to_move
                else:
                    next_pos1 = arms[i].angles
                    thirdJVel = 0
                print(next_pos1)
                arms[i].set_servo_angle_j(angles=next_pos1, is_radian=False)
                currPos[i] = next_pos1
                currVel[i] = [0,0,thirdJVel,0,0,0,0]

        # print(tts)
        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)


if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring

    IP0 = initPosPoppingOne
    IPstring = [IP0]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5006
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.237') # middle left
    global arms
    arms = [arm1]
    # arms = [arm1]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    visionThread = Thread(target=visionThread)
    robotThread = Thread(target=robotThread)
    visionThread.start()
    robotThread.start()

    while True:
        input(".")


