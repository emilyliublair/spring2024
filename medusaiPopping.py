import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue

# global vars
#arm 1
queueOne = Queue()
initPosPoppingOne = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
finPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
currPosOne = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
prevStatusOne = "back"
counterOne = 0
statusOne = False

#arm 2
queueTwo = Queue()
initPosPoppingTwo = [0.0, 30.0, 0.0, 90.0, 0.0, 50.0, 0.0]
finPosPoppingTwo = [0.0, 60.0, 0.0, 130.0, 0.0, 0.0, 0.0]
currPosTwo = [0.0, 30.0, 0.0, 90.0, 0.0, 50.0, 0.0]
currVelTwo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeTwo = 0
prevStatusTwo = "back"
counterTwo = 0
statusTwo = False

queue = [queueOne, queueTwo]
initPosPopping = [initPosPoppingOne, initPosPoppingTwo]
finPosPopping = [finPosPoppingOne, finPosPoppingTwo]
currPos = [currPosOne, currPosTwo]
currVel = [currVelOne, currVelTwo]
currTime = [currTimeOne, currTimeTwo]
prevStatus = [prevStatusOne, prevStatusTwo]
counter = [counterOne, counterTwo]
status = [statusOne, statusTwo]

no_one_tracker = True

def strumbot(traj):
    # j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)

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

def motifRecognizer(name,*args):
    print("got")
    receivedm = np.array(list(args))
    print(receivedm)

def distance_point_to_line_segment(point, segment_start, segment_end):
    point = np.array(point)
    segment_start = np.array(segment_start)
    segment_end = np.array(segment_end)

    segment_vector = segment_end - segment_start
    point_vector = point - segment_start

    t = np.dot(point_vector, segment_vector) / np.dot(segment_vector, segment_vector)
    t = max(0, min(1, t))
    closest_point = segment_start + t * segment_vector
    distance = np.linalg.norm(point - closest_point)

    return distance

def visionThread():
    global no_one_tracker
    global arms
    global counter
    global prevStatus
    global status

    while True:
        data, addr = sock.recvfrom(1024)
        # print(data.decode("utf-8"))
        if ("noone" in data.decode("utf-8")):
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
            armOneDetect = False
            armTwoDetect = False
            for i in range(len(rec_arr)):
                x_coord = float(rec_arr[i].split(",")[0])
                if x_coord <= 330:
                    armOneDetect = True
                    if prevStatus[0] == "out":
                        counter[0] += 1
                    else:
                        counter[0] = 0
                        prevStatus[0] = "out"
                    if counter[0] > 5 and not(status[0]):
                        status[0] = True
                        print("arm1) out")
                        pose_to_pose(currPos[0], finPosPopping[0], currVel[0], 0, 5-currTime[0], 0)
                elif x_coord > 330:
                    armTwoDetect = True
                    if prevStatus[1] == "out":
                        counter[1] += 1
                    else:
                        counter[1] = 0
                        prevStatus[1] = "out"
                    if counter[1] > 5 and not(status[1]):
                        status[1] = True
                        print("arm2) out")
                        pose_to_pose(currPos[1], finPosPopping[1], currVel[1], 0, 5-currTime[1], 1)

            if not(armOneDetect):
                if prevStatus[0] == "back":
                    counter[0] += 1
                else:
                    counter[0] = 0
                    prevStatus[0] = "back"
                if counter[0] > 5 and status[0]:
                    status[0] = False
                    print("arm1) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[0], initPosPopping[0], currVel[0], 0, currTime[0], 0)
            if not(armTwoDetect):
                if prevStatus[1] == "back":
                    counter[1] += 1
                else:
                    counter[1] = 0
                    prevStatus[1] = "back"
                if counter[1] > 5 and status[1]:
                    status[1] = False
                    print("arm2) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[1], initPosPopping[1], currVel[1], 0, currTime[1], 1)

def robotThread(armNum):
    while True:
        if not(queue[armNum].empty()):
            info = queue[armNum].get()

            next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

            next_time = info[2]
            next_vel = info[4]
            print(str(armNum) + str(next_pos))
            arms[armNum].set_servo_angle_j(angles=next_pos, is_radian=False)

            currPos[armNum] = next_pos
            currTime[armNum] = next_time
            currVel[armNum] = next_vel

            start_time = time.time()
            tts = time.time() - start_time

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

    IP1 = initPosPoppingOne
    IP2 = initPosPoppingTwo
    IPstring = [IP1, IP2]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.237')
    arm2 = XArmAPI('192.168.1.204')
    global arms
    arms = [arm1, arm2]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    visionThread = Thread(target=visionThread)
    robotOneThread = Thread(target=robotThread, args=(0,))
    robotTwoThread = Thread(target=robotThread, args=(1,))
    visionThread.start()
    robotOneThread.start()
    robotTwoThread.start()

    while True:
        input(".")


    #we want interrupt to happen at time x
    #get the vel at time x would be trajvel[x/0.004] = vinterrupt
    #get the pos at time x would be trajpos[x/0.004] = qinterrupt
    #make new trajectory fifth_poly(qinterrupt, q_f, vinterrupt, vf still 0, time from when you interrupt to new position):
    # vision thread puts distance as q.put('distance')
    #robot thread reads distance as dist = q.get()
