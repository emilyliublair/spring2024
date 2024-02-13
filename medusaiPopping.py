import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue

# global vars
time_to_move = 3
counter_threshold = 3

amp = 10
iteration_t = 5
traj_t = np.arange(0, 10, 0.004)

#snaking pos
two_in_p = amp * np.sin((np.pi / iteration_t) * traj_t)
four_in_p = amp * np.sin((np.pi / iteration_t) * traj_t)
six_in_p = amp * np.sin((np.pi / iteration_t) * traj_t)

two_out_p = amp * np.sin((np.pi / iteration_t) * traj_t)
four_out_p = amp * np.sin((np.pi / iteration_t) * traj_t)
six_out_p = amp * np.sin((np.pi / iteration_t) * traj_t)

#snaking vel
two_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
four_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
six_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)

two_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
four_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
six_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)


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
snaking_tracker_one = 0

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
snaking_tracker_two = 0

#arm 3
queueThree = Queue()
initPosPoppingThree = [124.1, 3.7, 8.7, 118.4, -13.9, 30.7, 0.0]
finPosPoppingThree = [124.1, 31.1, 8.7, 102.0, -13.9, -31.0, 0.0]
currPosThree = [124.1, 3.7, 8.7, 118.4, -13.9, 30.7, 0.0]
currVelThree = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeThree = 0
prevStatusThree = "back"
counterThree = 0
statusThree = False
snaking_tracker_three = 0

queue = [queueOne, queueTwo, queueThree]
initPosPopping = [initPosPoppingOne, initPosPoppingTwo, initPosPoppingThree]
finPosPopping = [finPosPoppingOne, finPosPoppingTwo, finPosPoppingThree]
currPos = [currPosOne, currPosTwo, currPosThree]
currVel = [currVelOne, currVelTwo, currVelThree]
currTime = [currTimeOne, currTimeTwo, currTimeThree]
prevStatus = [prevStatusOne, prevStatusTwo, prevStatusThree]
counter = [counterOne, counterTwo, counterThree]
status = [statusOne, statusTwo, statusThree]
snaking_tracker = [snaking_tracker_one, snaking_tracker_two, snaking_tracker_three]

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
            armThreeDetect = False
            for i in range(len(rec_arr)):
                x_coord = float(rec_arr[i].split(",")[0])
                if x_coord < 260:
                    armOneDetect = True
                    if prevStatus[0] == "out":
                        counter[0] += 1
                    else:
                        counter[0] = 0
                        prevStatus[0] = "out"
                    if counter[0] > counter_threshold and not(status[0]):
                        status[0] = True
                        print("arm1) out")
                        pose_to_pose(currPos[0], finPosPopping[0], currVel[0], 0, time_to_move-currTime[0], 0)
                if x_coord > 380:
                    armTwoDetect = True
                    if prevStatus[1] == "out":
                        counter[1] += 1
                    else:
                        counter[1] = 0
                        prevStatus[1] = "out"
                    if counter[1] > counter_threshold and not(status[1]):
                        status[1] = True
                        print("arm2) out")
                        pose_to_pose(currPos[1], finPosPopping[1], currVel[1], 0, time_to_move-currTime[1], 1)
                if 210 <= x_coord <= 440:
                    armThreeDetect = True
                    if prevStatus[2] == "out":
                        counter[2] += 1
                    else:
                        counter[2] = 0
                        prevStatus[2] = "out"
                    if counter[2] > counter_threshold and not(status[2]):
                        status[2] = True
                        print("arm3) out")
                        pose_to_pose(currPos[2], finPosPopping[2], currVel[2], 0, time_to_move-currTime[2], 2)
            if not(armOneDetect):
                if prevStatus[0] == "back":
                    counter[0] += 1
                else:
                    counter[0] = 0
                    prevStatus[0] = "back"
                if counter[0] > counter_threshold and status[0]:
                    status[0] = False
                    print("arm1) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[0], initPosPopping[0], currVel[0], 0, currTime[0], 0)
            if not(armTwoDetect):
                if prevStatus[1] == "back":
                    counter[1] += 1
                else:
                    counter[1] = 0
                    prevStatus[1] = "back"
                if counter[1] > counter_threshold and status[1]:
                    status[1] = False
                    print("arm2) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[1], initPosPopping[1], currVel[1], 0, currTime[1], 1)
            if not(armThreeDetect):
                if prevStatus[2] == "back":
                    counter[2] += 1
                else:
                    counter[2] = 0
                    prevStatus[2] = "back"
                if counter[2] > counter_threshold and status[2]:
                    status[2] = False
                    print("arm3) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[2], initPosPopping[2], currVel[2], 0, currTime[2], 2)

def robotThread():
    global snaking_tracker_one
    global snaking_tracker_two
    global snaking_tracker_three

    while True:
        start_time = time.time()
        tts = time.time() - start_time
        for i in range(len(queue)):
            if not(queue[i].empty()):
                snaking_tracker[i] = 0
                info = queue[i].get()
                next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1),
                            round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

                arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
                currPos[i] = next_pos
                currTime[i] = info[2]
                currVel[i] = info[4]
            else:
                if (status[i]):
                    next_pos = arms[i].angles
                    next_pos[1] = two_out_p[snaking_tracker[i] % 2500] + finPosPopping[i][1]
                    next_pos[3] = two_out_p[snaking_tracker[i] % 2500] + finPosPopping[i][3]
                    next_pos[5] = two_out_p[snaking_tracker[i] % 2500] + finPosPopping[i][5]
                    currVel[i] = [0.0, two_out_v[snaking_tracker[i] % 2500], 0.0,
                                  four_out_v[snaking_tracker[i] % 2500], 0.0, six_out_v[snaking_tracker[i] % 2500],
                                  0.0]
                else:
                    next_pos = arms[i].angles
                    next_pos[1] = two_in_p[snaking_tracker[i] % 2500] + initPosPopping[i][1]
                    next_pos[3] = four_in_p[snaking_tracker[i] % 2500] + initPosPopping[i][3]
                    next_pos[5] = six_in_p[snaking_tracker[i] % 2500] + initPosPopping[i][5]
                    currVel[i] = [0.0, two_in_v[snaking_tracker[i] % 2500], 0.0, four_in_v[snaking_tracker[i] % 2500],
                                  0.0, six_in_v[snaking_tracker[i] % 2500], 0.0]

                arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
                currPos[i] = next_pos
                snaking_tracker[i] += 1

        # if not(queue[0].empty()):
        #     snaking_tracker_one = 0
        #     info = queue[0].get()
        #
        #     next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]
        #     # next_time = info[2]
        #     # next_vel = info[4]
        #     print(str(0) + str(next_pos))
        #     arms[0].set_servo_angle_j(angles=next_pos, is_radian=False)
        #
        #     currPos[0] = next_pos
        #     currTime[0] = info[2]
        #     currVel[0] = info[4]
        # else:
        #     if (status[0]):
        #         next_pos = arms[0].angles
        #         next_pos[1] = two_out_p[snaking_tracker_one%2500] + finPosPopping[0][1]
        #         next_pos[3] = two_out_p[snaking_tracker_one % 2500] + finPosPopping[0][3]
        #         next_pos[5] = two_out_p[snaking_tracker_one % 2500] + finPosPopping[0][5]
        #         currVel[0] = [0.0, two_out_v[snaking_tracker_one % 2500], 0.0, four_out_v[snaking_tracker_one % 2500], 0.0, six_out_v[snaking_tracker_one % 2500], 0.0]
        #     else:
        #         next_pos = arms[0].angles
        #         next_pos[1] = two_in_p[snaking_tracker_one % 2500] + initPosPopping[0][1]
        #         next_pos[3] = four_in_p[snaking_tracker_one % 2500] + initPosPopping[0][3]
        #         next_pos[5] = six_in_p[snaking_tracker_one % 2500] + initPosPopping[0][5]
        #         currVel[0] = [0.0, two_in_v[snaking_tracker_one % 2500], 0.0, four_in_v[snaking_tracker_one % 2500], 0.0, six_in_v[snaking_tracker_one % 2500], 0.0]
        #
        #     arms[0].set_servo_angle_j(angles=next_pos, is_radian=False)
        #     currPos[0] = next_pos
        #     snaking_tracker_one += 1
        #
        # if not(queue[1].empty()):
        #     snaking_tracker_two = 0
        #     info = queue[1].get()
        #
        #     next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]
        #
        #     print(str(1) + str(next_pos))
        #     arms[1].set_servo_angle_j(angles=next_pos, is_radian=False)
        #
        #     currPos[1] = next_pos
        #     currTime[1] = info[2]
        #     currVel[1] = info[4]
        #
        # if not(queue[2].empty()):
        #     snaking_tracker_three = 0
        #     info = queue[2].get()
        #
        #     next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]
        #
        #     print(str(2) + str(next_pos))
        #     arms[2].set_servo_angle_j(angles=next_pos, is_radian=False)
        #
        #     currPos[2] = next_pos
        #     currTime[2] = info[2]
        #     currVel[2] = info[4]

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
    IP1 = initPosPoppingTwo
    IP2 = initPosPoppingThree
    IPstring = [IP0, IP1, IP2]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.237') # left front
    arm2 = XArmAPI('192.168.1.204') # right front
    arm3 = XArmAPI('192.168.1.244') # middle front
    global arms
    arms = [arm1, arm2, arm3]

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


    #we want interrupt to happen at time x
    #get the vel at time x would be trajvel[x/0.004] = vinterrupt
    #get the pos at time x would be trajpos[x/0.004] = qinterrupt
    #make new trajectory fifth_poly(qinterrupt, q_f, vinterrupt, vf still 0, time from when you interrupt to new position):
    # vision thread puts distance as q.put('distance')
    #robot thread reads distance as dist = q.get()
