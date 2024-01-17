import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue

# global vars
status = False
queue = Queue()
initPosPopping = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
finPosPopping = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
line_start = [227, 219]
line_end = [39, 359]

currPos = None
currVel = None
currTime = None

no_one_tracker = False
status = False

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
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring.copy()
        # angle[0] = angle[0]- strumD/2
        a.set_servo_angle(angle=angle, wait=True, speed=10, acceleration=0.25, is_radian=False)

def pose_to_pose(initPos, finPos, vi, vf, t):
    global queue
    with queue.mutex:
        queue.queue.clear()
        queue.all_tasks_done.notify_all()
        queue.unfinished_tasks = 0
    currArmAngles = [round(arms[0].angles[0], 1), round(arms[0].angles[1], 1), round(arms[0].angles[2], 1), round(arms[0].angles[3], 1), round(arms[0].angles[4], 1), round(arms[0].angles[5], 1), round(arms[0].angles[6], 1)]
    validInitPos = True
    for i in range(len(initPos)):
        if currArmAngles[i] == initPos[i]: continue
        diff = abs(initPos[i]-currArmAngles[i])
        if diff > 5:
            print(initPos[i])
            print(currArmAngles[i])
            print(diff)
            print(i)
            validInitPos = False
            break
    if validInitPos:
        posTrajs = []
        velTrajs = []
        for i in range(len(initPos)): # generate trajs and vels for each joint
            tempTraj, tempVel = fifth_poly(initPos[i], finPos[i], vi[i], vf, t) # tempTraj, tempVel has t/.004 length
            posTrajs.append(tempTraj)
            velTrajs.append(tempVel)
        reformatPos = [currArmAngles]
        reformatVel = [currVel]
        for i in range(len(posTrajs[0])):
            posRow = []
            velRow = []
            for j in range(7):
                posRow.append(posTrajs[j][i])
                velRow.append(velTrajs[j][i])
            queue.put([reformatPos[-1], posRow, 0.004 + (i * 0.004), reformatVel[-1], velRow]) # each queue item has prev_pos, next_pos, time, prev_vel, next_vel for all joints
            reformatPos.append(posRow)
            reformatVel.append(velRow)
    else:
        print(initPos)
        print(currArmAngles)
        print("init pos not the same")

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

def visionThread(line_start, line_end):
    global no_one_tracker
    global status
    while True:
        data, addr = sock.recvfrom(1024)
        if ("noone" in data.decode("utf-8")):
            status = False
            if currPos != initPosPopping and not(no_one_tracker):
                pose_to_pose(currPos, initPosPopping, currVel, 0, currTime)
                no_one_tracker = True
                print("go back, no one detected")
        else:
            no_one_tracker = False
            arr = ast.literal_eval(data.decode("utf-8"))
            temp_detected = False
            for i in range(len(arr)):
                coords = [float(arr[i].split(",")[0]), float(arr[i].split(",")[1].strip())]
                dist = distance_point_to_line_segment(coords, line_start, line_end)
                if dist < 30: # detected
                    temp_detected = True
                    if not(status):
                        status = True
                        pose_to_pose(currPos, finPosPopping, currVel, 0, 5-currTime)
                        print("go out")

            if not(temp_detected) and status:
                status = False
                pose_to_pose(currPos, initPosPopping, currVel, 0, currTime)
                print("go back")
def robotThread():
    global currPos
    global currTime
    global currVel
    while True:
        if not(queue.empty()):
            info = queue.get()
            curr_pos = [round(info[0][0], 1), round(info[0][1], 1), round(info[0][2], 1), round(info[0][3], 1), round(info[0][4], 1), round(info[0][5], 1), round(info[0][6], 1)]
            next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

            next_time = info[2]
            next_vel = info[4]

            if curr_pos == currPos:
                arms[0].set_servo_angle_j(angles=next_pos, is_radian=False)

                currPos = next_pos
                currTime = next_time
                currVel = next_vel
            else:
                print("robot thread")
                print(curr_pos)
                print(currPos)

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
    IPstring = [0.0, -30.0, 0.0, 125.0, 0.0, 11.0, -45.0]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.236')
    global arms
    arms = [arm1]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    currPos = IPstring
    currVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    currTime = 0

    visionThread = Thread(target=visionThread, args=(line_start, line_end))
    robotThread = Thread(target=robotThread)
    visionThread.start()
    robotThread.start()

    pose_to_pose(IPstring, initPosPopping, currVel, 0, 2)
    while True:
        input(".")


    #we want interrupt to happen at time x
    #get the vel at time x would be trajvel[x/0.004] = vinterrupt
    #get the pos at time x would be trajpos[x/0.004] = qinterrupt
    #make new trajectory fifth_poly(qinterrupt, q_f, vinterrupt, vf still 0, time from when you interrupt to new position):
    # vision thread puts distance as q.put('distance')
    #robot thread reads distance as dist = q.get()
