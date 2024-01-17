import os
import sys
import time
import numpy as np
import math
from xarm.wrapper import XArmAPI
import argparse
from pythonosc import udp_client

def strumbot(traj):
    # j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        # print(j_angles)
        while tts < 0.004:
            # print(tts)
            tts = time.time() - start_time
            time.sleep(0.0001)


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
        # a.set_servo_angle(angle=angle, wait=False, speed=20, acceleration=0.5, is_radian=False)


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
    return traj_pos


def matlabparser(traj, v_i, v_f, IP):
    # format is [[[pos1,pos2],[time1,time2]],[[JOINT2pos1,pos2],[time1,time2]]]
    trajblock = []
    j = 0
    for joint in traj:
        # format is now at [[pos1,pos2],[time1,time2]]
        jointblock = np.empty([0, 1])
        if len(joint[0]) == 1:
            iter = 1
        else:
            iter = len(joint[0])
        for i in range(iter):
            if i == 0:
                v = v_i[j]
                vf = 0
                qi = IP[j]
                qf = joint[0][i]
            elif i == iter - 1:
                v = 0
                vf = v_f[j]
                qi = joint[0][i - 1]
                qf = joint[0][i]
            else:
                v = 0
                vf = 0
                qi = joint[0][i - 1]
                qf = joint[0][i]
            timescale = strumtime * joint[1][i] / sum(joint[1])
            step = fifth_poly(qi, qf, v, vf, timescale)
            jointblock = np.append(jointblock, step)
        # print("JOINT BLOCK",j,len(jointblock))
        trajblock.append(jointblock)
        j += 1
    # print(len(trajblock[3]))
    # reformat
    reformat = []
    for i in range(len(trajblock[0])):
        row = []
        # for q in range(7):
        # print(len(trajblock[q]))
        for j in range(7):
            row.append(trajblock[j][i])
        reformat.append(row)
    return reformat


def dancevel(t):
    amp5 = headS * 8.6
    amp3 = 0.4 * amp5
    amp1 = 0.2 * amp5
    amp5extra = 30
    amptwist = 200
    headS5 = 0.045 * amptwist * headS
    headS3 = 0.09 * amptwist * headS

    j_angles = [0, 0, 0, 0, 0, 0, 0]
    strike = (math.pi / headS3) * (math.cos((math.pi / headS3) * (t)))
    two = (math.pi / headS) * math.cos(math.pi * (0.75 - (t / headS)))
    three = (math.pi / headS3) * (math.cos((math.pi / headS3) * (t)))
    four = (math.pi / headS) * math.cos(math.pi * (0.5 - (t / headS)))
    five = (math.pi / headS5) * math.cos((math.pi / headS5) * (t))
    six = (math.pi / headS) * math.cos((math.pi / headS) * (t))

    # j_angles[0] = -baseamp * strike
    j_angles[1] = -amp1 * two
    j_angles[2] = +amptwist * three
    j_angles[3] = +amp3 * four
    j_angles[4] = +amptwist * five
    j_angles[5] = -amp5 * six
    j_angles[6] = -(amptwist * five + amptwist * three - baseamp * strike)
    return j_angles


def dancepos(startp, t, phase):
    amp5 = headS * 8.6
    amp3 = 0.4 * amp5
    amp1 = 0.2 * amp5
    amp5extra = 30
    amptwist = 200
    headS5 = 0.045 * amptwist * headS
    headS3 = 0.09 * amptwist * headS

    j_angles = startp.copy()
    strike = (math.sin((math.pi / headS3) * (t)))
    two = math.sin((math.pi / headS) * (t - 0.75*headS-phase*headS))
    three = math.sin((math.pi / headS3) * (t-phase*headS3))
    four = math.sin((math.pi / headS) * (t-0.5*headS-phase*headS))
    five = math.sin((math.pi / headS5) * (t-phase*headS5))
    six = math.sin((math.pi / (headS) * (t-phase*headS)))

    # j_angles[0] = startp[0] - baseamp * strike
    j_angles[1] = startp[1] - amp1 * two
    j_angles[2] = startp[2] + amptwist * three
    j_angles[3] = startp[3] + amp3 * four
    j_angles[4] = startp[4] + amptwist * five
    j_angles[5] = startp[5] - amp5 * six
    j_angles[6] = startp[6] - (amptwist * five + amptwist * three - baseamp * strike)
    if abs(j_angles[6]) >= 360:
        j_angles[6] = (j_angles[6]/abs(j_angles[6]))*359.8
    return j_angles


if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring
    IP1 = [0.0, -30, 0.0, 125, 0.0, 11.0, -45]
    IP2 =[0.0, -8.5, 0, 95, 0, 30, -45]
    IP3 = [34.0, 12, 0, 90, 0, 30, -45]
    IP4 = [144, 20, 0, 120, 0, 40, -45]
    IP5 = [-13.3, 46, 0, 113, 0, 10, -45]
    IP6 = [-83.6, -49, 0, 105, -130, 10, -45]
    IPstring = [IP1, IP2, IP3, IP4, IP5, IP6]

    # IP0u = IPstring.copy()
    # IP0u[2] = IP0u[2]-baseamp/2
    # IP0u[3] = IP0u[3] + uamp
    # IP0u[0] = IP0u[0] - (baseamp/(scaledbase*2))
    # strumto start traj is = [0, 7.7, 0, 111.72, 9, 11, -45]

    arm1 = XArmAPI('192.168.1.236')

    global arms
    arms = [arm1]
    phases = [0, 0.5, 0.75, 1, 0.75, .75]
    parser = argparse.ArgumentParser()

    parser.add_argument("--ip", default="192.168.2.8", help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=5004, help="The port the OSC server is listening on")
    args = parser.parse_args()
    client = udp_client.SimpleUDPClient(args.ip, args.port)

    # while True:
    #     client.send_message("/arm{}".format(0), [0, 1, 2, 90, 4, 5, 6])
    #     input("next")
    #     client.send_message("/arm{}".format(0), [7, 8, 9, 120, 10, 11, 12])


    # pos2 = [39.5, 84.5, -40, 96.6,-strumD / 2, 49.3, -31.2]

    # totalArms = len(arms)

    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    setup(2)

    # input("test simulation")
    # for traj in utrajectory:
    #     time.sleep(0.004)
    #     print(traj)

    # print(pos)
    # print(pos[joint]-strumD/2)
    input("test strumming")

    global headS
    global strumtime

    timechanger = 2

    strumtime = 18
    headS = 2
    arm1.set_mode(0)
    arm1.set_state(0)
    prepTime = (2 * (2 / headS))

    # [0, 7.7, 0, 111.72, 9, 11, -45]
    overlap = headS*18
    strumIP1 = dancepos(IPstring[0], (overlap-prepTime), 0)
    endp = dancepos(IPstring[0], overlap, 0)
    strumD = 15
    center = 52.4
    head1 = center -strumD/2
    head2 = center + strumD/2
    struminput1 = [[[6, 4.7, 4.7, 0, endp[0]], [0.75, 0.25, 2, 1, 5]], [[-27.6, -56.38, -56.38, endp[1]], [1, 1, 2, 5]],
                   [[6, 4.7, 4.7, endp[2]], [0.75, 0.25, 6, 2]], [[118.1, 35.75, 35.75, 100, endp[3]], [1.5, 1, 2, 2.5, 2.5]],
                   [[0, 0, 70, endp[4]], [2, 3, 2, 2]], [[head1, head1, head2, -10, endp[5]], [1, 1.5, 0.3, 3.5+0.7, 2]],
                   [[-35.9, -35.9, endp[6]], [1, 8.5, 0.5]]]

    #[4.7, -56.44, 4.7, 35.7, 0, -strumD / 2 + 52.4, -20.9] STRUM D IS 15 and happens in 1/8 of second


    # OLD DONT USE struminput1 = [ [[20.5,20.5,50,endp[0]],[1.5,1.5,1,5]], [[-55.9,-55.9,endp[1]],[3,1,5]], [[-4.4,33,endp[2]],[2,4,3]], [[70,12.8,14.8, 125,endp[3]],[1.5,2,1,2.5,2.5]], [[0,0,70,endp[4]],[2,3,2,2]], [[60,30,30,-10,endp[5]],[2,2,0.5,1.5,3]], [[-45,-45,endp[6]],[1,8.5,0.5]] ]
    vstrum1 = dancevel(overlap-prepTime)
    vfstrum1 = dancevel(overlap)
    strumtraj1 = matlabparser(struminput1,vstrum1,vfstrum1,strumIP1)
    #
    strumIP2 = dancepos(IPstring[0],((overlap*2)-prepTime), 0)
    endp2 = dancepos(IPstring[0], 2*overlap, 0)
    struminput2 = [[[6, 4.7, 4.7, 0, endp2[0]], [0.75, 0.25, 2, 1, 5]], [[-27.6, -56.38, -56.38, endp2[1]], [1, 1, 2, 5]],
                   [[6, 4.7, 4.7, endp2[2]], [0.75, 0.25, 6, 2]], [[118.1, 35.75, 35.75, 100, endp2[3]], [1.5, 1, 2, 2.5, 2.5]],
                   [[0, 0, 70, endp2[4]], [2, 3, 2, 2]], [[head1, head1, head2, -10, endp2[5]], [1, 1.5, 0.3, 3.5+0.7, 2]],
                   [[-20.9, -20.9, endp2[6]], [1, 8.5, 0.5]]]

    # struminput2 = [ [[14,13.3,30,endp[0]],[2,2,1,4]], [[7.7,-55.9,-55.9,endp[1]],[1.5,2,0.5,5]], [[0,33,endp[2]],[2,4,3]], [[40,40,13.8,15.8, 125,endp[3]],[2,1,1,1,2.5,1.5]], [[0,0,70,endp[4]],[2,3,2,2]], [[60,30,30,-10,endp[5]],[2,2,0.5,1.5,3]], [[-45,-45,endp[6]],[1,8.5,0.5]] ]
    vstrum2 = dancevel((overlap*2)-prepTime)
    vfstrum2 = dancevel(overlap*2)
    strumtraj2 = matlabparser(struminput2,vstrum2,vfstrum2,strumIP2)
    #
    strums =[strumtraj1, strumtraj2]

    starting = []

    for i in range(len(arms)):
        starting.append(dancepos(IPstring[i], 0, phases[i]))
        print(dancepos(IPstring[i], 0, phases[i]))

    for armnum in range(len(arms)):
        arms[armnum].set_servo_angle(angle=starting[armnum], wait=True, speed=20, acceleration=0.5,
                                     is_radian=False)
        # client.send_message("/arm{}".format(armnum), starting[armnum])

    input("begin strum")

    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    i = 0
    amp = uamp

    t = 0.0
    input("start")
    time.sleep(5)
    strumcount = 0
    print("started")
    while True:
        start_time = time.time()
        # j_angles[0] = pose[0] + 0.5 * (baseamp/scaledbase) * strike + 0.5 * (baseamp/scaledbase)
        goto = []
        for i in range(len(arms)):
            goto.append(dancepos(IPstring[i], t, phases[i]))
            # print(dancepos(IPstring[i], 0, phases[i]))

        t += 0.004
        tts = time.time() - start_time

        # print(t%36)
        if abs((t % overlap)-(overlap-prepTime))<0.001:
            client.send_message("/strum", [1])
            print("STRUM")

            strumdir = strumcount % 2
            print(strumdir)
            strumbot(strums[strumdir])
            strumcount += 1
            t+=prepTime
        # print(goto[0])
        for armnum in range(len(arms)):
            arms[armnum].set_servo_angle_j(angles=goto[armnum], is_radian=False)
            # client.send_message("/arm{}".format(armnum), goto[armnum])
        # arm1.set_servo_angle_j(angles=goto, is_radian=False)
        while tts < 0.004:
            # print(tts)
            tts = time.time() - start_time
            time.sleep(0.0001)
        # print(tts)


