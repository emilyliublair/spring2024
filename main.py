import math
import numpy as np
from ultralytics import YOLO
# from ultralytics.yolo.utils.plotting import Annotator
import cv2, glob
import mediapipe as mp
from pythonosc import udp_client, dispatcher, osc_server
from pythonosc import osc_message_builder
import socket
from collections import defaultdict
import time
from sympy import Point, Line

UDP_IP_self = "127.0.0.1"
UDP_IP_Nic = "192.168.1.50"  # local IP
UDP_IP_Rip = "192.168.2.8"  # local IP
# # UDP_IP = "0.0.0.0"
UDP_PORT_Nic = 5002  # port to retrieve data from Max
UDP_PORT_Rip = 5003  # port to retrieve data from Max
UDP_PORT_self = 5005
# 
# # sock.sendto(b"hi", (UDP_IP, UDP_PORT))
client_Nic = udp_client.UDPClient(UDP_IP_Nic, UDP_PORT_Nic)
client_Rip = udp_client.UDPClient(UDP_IP_Rip, UDP_PORT_Rip)
selfSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# client_self = udp_client.UDPClient(UDP_IP_self, UDP_PORT_self)

# client.send("/coordiante", 1)
# print("server up")
# print(sock.sendto(b"send_coordinate", (UDP_IP, UDP_PORT)))

model = YOLO('yolov8n-pose.pt')
# mp_pose = mp.solutions.pose
# mp_drawing = mp.solutions.drawing_utils

video_path = 0
cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(2)

# velocity calculation
old_wing = None
speed = [0,0,0,0,0,0]
prev_time_frame = 0
new_time_frame = 0
window = [[], [], [], [], [], []]

# sparse optical flow lucas kanade
p0 = None
p1 = None
st = np.array([[1],[1],[1],[1],[1],[1]])
old_frame = None
old_gray = None

# feature_params = dict( maxCorners = 100,
#  qualityLevel = 0.3,
#  minDistance = 7,
#  blockSize = 7 )
lk_params = dict( winSize = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                              10, 0.03))
color = np.random.randint(0, 255, (100, 3))

# track
track_history = defaultdict(lambda: [[], [], [], [], [], []])
wing_history = defaultdict(lambda: [])
coordinate_history = defaultdict(lambda: [])
coordinate_history_un = defaultdict(lambda: [])

# scaling
width = 640
height = 320

zz = (9, 199)
oz = (420,366)
zo = (171,134)
oo = (534,244)

pts1 = np.float32([zo, zz, oz, oo])
pts2 = np.float32([[0,0], [0, height], [width, height], [width, 0]])
matrix = cv2.getPerspectiveTransform(pts1, pts2)

distanceThreshold = 300

# pts1 = np.float32([[0, 416], [106, 164], [508, 130], [640, 318]])
# pts2 = np.float32([[0, height], [0, 0], [width, 0], [width, height]])
# matrix = cv2.getPerspectiveTransform(pts1, pts2)

# distance
# l2p1, l2p2 = np.asarray((227, 219)), np.asarray((333, 281))
ellipse_center = [350, 155]
# ellipse_center_1 = [359, 155, 1]
# ellipse_center_scaled = np.matmul(ellipse_center_1, matrix)
# axes_lengths = [414-362 + 10, 197-173]
# axes_lengths_1 = [414-362 + 10, 197-173, 1]
# axes_lengths_scaled = np.matmul(axes_lengths_1, matrix)
# start_angle = 0
# end_angle = 180
green = (0, 255, 0)

# cc_x = 320
# cc_y = 130
# circle_center = [cc_x, cc_y]
# circle_center_scaled = np.matmul([323, 172, 1], matrix)
# circle_radius = 110

p1 = (114, 79)
p2 = (590, 226)
line_slope = -.2938

vid_path = "./vid.mp4"
# cap = cv2.VideoCapture(vid_path)

def calcDistanceFromCircle(center, point, radius):
    return (((point[0]-center[0])**2 + (point[1]-center[1])**2) ** (1/2)) - radius

def calcDistanceFromEllipse(x, y):
    distance = math.sqrt((x - ellipse_center_scaled[0]) ** 2 + ((y - ellipse_center_scaled[1]) ** 2) / ((axes_lengths[0] ** 2) / (axes_lengths[1] ** 2)))
    return distance

def calcDistanceFromPoint(x, y):
    distance = math.sqrt((ellipse_center[0] - x) ** 2 + (ellipse_center[1]-y) ** 2)
    return distance

def withinBounds(x, y):
    line_y = line_slope * (x - p1[0]) + p1[1]
    return -y < line_y
    
# with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
while cap.isOpened():
    success, frame = cap.read()
    if success:
        ### reset tracking to conserve space
        if len(track_history) > 50:
            print("reset")
            track_history = defaultdict(lambda: [[], [], [], [], [], []])
            wing_history = defaultdict(lambda: [])
            coordinate_history = defaultdict(lambda: [])
            coordinate_history_un = defaultdict(lambda: [])
            old_wing = None
            speed = [0, 0, 0, 0, 0, 0]
            prev_time_frame = 0
            new_time_frame = 0
            window = [[], [], [], [], [], []]

        results = model.track(frame, save=False, persist=True, conf=0.65)
        annotated_frame = results[0].plot()

        # MEDUSA OUTLINE
        # cv2.circle(annotated_frame, ellipse_center, distanceThreshold, green, 1)
        # cv2.line(annotated_frame, p1, p2, green, 1)
        # cv2.ellipse(annotated_frame, ellipse_center, axes_lengths, 0, start_angle, 180, green, 1)
        # cv2.ellipse(annotated_frame, (cc_x, cc_y), (circle_radius, circle_radius), 0, 0, 180, (0, 0, 255), 3)

        # SCALING
        # cv2.line(annotated_frame, zz, oz, green, 1)
        # cv2.line(annotated_frame, oz, oo, green, 1)
        # cv2.line(annotated_frame, oo, zo, green, 1)
        # cv2.line(annotated_frame, zo, zz, green, 1)

        # # CENTER OF CAM
        # cv2.line(annotated_frame, (320, 0), (320, 480), (255, 255, 255), 1)
        # cv2.line(annotated_frame, (0, 240), (640, 240), (255, 255, 255), 1)
        #
        # # quadrants
        # cv2.line(annotated_frame, (227, 219), (39, 359), (255, 0, 0), 3)
        # cv2.line(annotated_frame, (433, 202), (639, 339), (255, 0, 0), 3)

        # scaling info
        ### LEFT
        # lh1p1 = (0, 420)
        # lh1p2 = (103, 168)
        # # lh1p1 = (24, 365)
        # # lh1p2 = (56, 281)
        # cv2.line(annotated_frame, lh1p1, lh1p2, (252, 3, 252), 1)
        #
        # rh1p1 = (519, 120)
        # rh1p2 = (640, 262)
        # cv2.line(annotated_frame, rh1p1, rh1p2, (252, 3, 252), 1)
        #
        # lh2p1 = (56, 280)
        # lh2p2 = (82, 217)
        # cv2.line(annotated_frame, lh2p1, lh2p2, (3, 252, 44), 1)
        #
        # lh3p1 = (82, 217)
        # lh3p2 = (105, 168)
        # cv2.line(annotated_frame, lh3p1, lh3p2, (252, 3, 252), 1)

        # # height line 1
        # h1p1 = (636, 409)
        # h1p2 = (577, 308)
        # cv2.line(annotated_frame, h1p1, h1p2, (252, 3, 252), 2)
        # h1Dist = calcDistanceFromCircle(h1p1, h1p2, 0)
        #
        # # height line 2
        # h2p1 = (577, 308)
        # h2p2 = (528, 232)
        # cv2.line(annotated_frame, h2p1, h2p2, (3, 252, 44), 2)
        # h2Dist = calcDistanceFromCircle(h2p1, h2p2, 0)
        #
        # # height line 3
        # h3p1 = (528, 232)
        # h3p2 = (491, 176)
        # cv2.line(annotated_frame, h3p1, h3p2, (252, 3, 252), 2)
        # h3Dist = calcDistanceFromCircle(h3p1, h3p2, 0)
        #
        # # height line 4
        # h4p1 = (491, 176)
        # h4p2 = (463, 135)
        # cv2.line(annotated_frame, h4p1, h4p2, (3, 252, 44), 2)
        # h4Dist = calcDistanceFromCircle(h4p1, h4p2, 0)

        # fps
        new_time_frame = time.time()
        fps = int(1 / (new_time_frame - prev_time_frame))
        prev_time_frame = new_time_frame
        cv2.putText(annotated_frame, "fps: " + str(fps), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        lb = (1, 416)
        rb = (640, 318)
        rt = (508, 130)
        lt = (106, 164)
        # cv2.line(annotated_frame, lb, rb, (255, 0, 0), 1)
        # cv2.line(annotated_frame, rb, rt, (255, 0, 0), 1)
        # cv2.line(annotated_frame, lb, lt, (255, 0, 0), 1)

        lh1p1 = (0, 420)
        lh1p2 = (103, 168)
        rh1p1 = (519, 120)
        rh1p2 = (640, 262)

        # rect sight
        tl = (208, 206)
        tr = (247, 234)
        bl = (29, 342)
        br = (55, 375)
        # cv2.line(annotated_frame, tl, tr, (0, 255, 0), 1)
        # cv2.line(annotated_frame, bl, tl, (0, 255, 0), 1)
        # cv2.line(annotated_frame, br, tr, (0, 255, 0), 1)
        # cv2.line(annotated_frame, bl, br, (0, 255, 0), 1)

        cv2.circle(annotated_frame, ellipse_center, radius=0, color=(0, 255, 0), thickness=-1)

        cv2.imshow("YOLOv8 Tracking", annotated_frame)
        # result = cv2.warpPerspective(annotated_frame, matrix, (width, height))
        # cv2.imshow("scaled", result)

        try:

            boxes = results[0].boxes.xywh.cpu()
            keypoints = results[0].keypoints
            track_ids = results[0].boxes.id.int().cpu().tolist()
            num_people_detected = len(track_ids)
            numPBytes = num_people_detected.to_bytes(4,'big')
            # selfSock.sendto(numPBytes, (UDP_IP_self, UDP_PORT_self))
            # selfSock.sendto(str.encode("detected"), (UDP_IP_self, UDP_PORT_self))
            counter = 0

            send_arr = []

            for box, kps, track_id in zip(boxes, keypoints, track_ids):
                send = True

                window = track_history[track_id]
                old_wing = wing_history[track_id]
                coordinate = coordinate_history[track_id]
                coordinate_un = coordinate_history_un[track_id]

                kp = kps.xy.tolist()[0]
                kp_n = kps.xyn.tolist()[0]

                left_shoulder = kp[5]
                left_shoulder.append(1)
                left_shoulder = np.matmul(left_shoulder, matrix)
                left_elbow = kp[7]
                left_elbow.append(1)
                left_elbow = np.matmul(left_elbow, matrix)
                left_hand = kp[9]
                left_hand.append(1)
                left_hand = np.matmul(left_hand, matrix)
                right_shoulder = kp[6]
                right_shoulder.append(1)
                right_shoulder = np.matmul(right_shoulder, matrix)
                right_elbow = kp[8]
                right_elbow.append(1)
                right_elbow = np.matmul(right_elbow, matrix)
                right_hand = kp[10]
                right_hand.append(1)
                right_hand = np.matmul(right_shoulder, matrix)

                left_foot = kp_n[15]
                right_foot = kp_n[16]
                left_foot_un = kp[15]
                right_foot_un = kp[16]
                left_foot_scaled = left_foot_un
                left_foot_scaled.append(1)
                left_foot_scaled = np.matmul(left_foot_scaled, matrix)
                right_foot_scaled = right_foot_un
                right_foot_scaled.append(1)
                right_foot_scaled = np.matmul(right_foot_scaled, matrix)

                window[0].append(left_shoulder)
                window[1].append(right_shoulder)
                window[2].append(left_elbow)
                window[3].append(right_elbow)
                window[4].append(left_hand)
                window[5].append(right_hand)

                # coordinate.append(right_shoulder_normalized)
                
                pos_scaled = (np.array([left_foot_scaled[0], left_foot_scaled[1]]) + np.array([right_foot_scaled[0], right_foot_scaled[1]])) / 2
                pos_un = (np.array(left_foot_un) + np.array(right_foot_un)) / 2
                pos = (np.array(left_foot) + np.array(right_foot))/2
                coordinate.append(pos)
                coordinate_un.append(pos_un)
                print(pos_un)
                
                if withinBounds(pos_un[0], pos_un[1]):

                ### CALCULATE DISTANCE
                    if len(coordinate) >= 3:
                        send_coordinate = np.mean(np.array(coordinate), axis=0)
                        send_coordinate_un = np.mean(np.array(coordinate_un), axis=0)
                        shortestDistance = calcDistanceFromPoint(pos_un[0], pos_un[1])
                        if shortestDistance < distanceThreshold:
                            # print(pos_un)
                            # print("distance: " + str(shortestDistance))
                        # shortestDistance = calcDistanceFromCircle([circle_center_scaled[0], circle_center_scaled[1]], pos_scaled, 0)
                        # sock.sendto(str.encode("dist: " + str(shortestDistance)), (UDP_IP_Hope, UDP_PORT_Hope))
    
                            msg = osc_message_builder.OscMessageBuilder(address="/shortDist" + str(counter))
                            msg.add_arg(shortestDistance)
                            msg = msg.build()
                            client_Rip.send(msg)
                            client_Nic.send(msg)
                            # # client_Hope.send(msg)
                            # 
                            msg2 = osc_message_builder.OscMessageBuilder(address="/position" + str(counter))
                            coordinate_msg = str(send_coordinate[0]) + ", " + str(send_coordinate[1])
                            coordinate_msg_un = str(send_coordinate_un[0]) + ", " + str(send_coordinate_un[1])
                            # # coordinate_msg_scaled = str(pos_scaled[0]) + ", " + str(pos_scaled[1])
                            msg2.add_arg(coordinate_msg)
                            msg2 = msg2.build()
                            client_Nic.send(msg2)
                            client_Rip.send(msg2)
                            send_arr.append(coordinate_msg_un)
    
                        coordinate.pop(0)
                        coordinate_un.pop(0)
                        
    
                    # new_wing = [np.average(np.array(window[0]), axis=0), np.average(np.array(window[1]), axis=0), np.average(np.array(window[2]), axis=0),
                    #             np.average(np.array(window[3]), axis=0), np.average(np.array(window[4]), axis=0), np.average(np.array(window[5]), axis=0)]
    
                    ### CALCULATE VELOCITY
                    # if len(window[0]) == 3:
                    #     if len(old_wing) > 0:
                    #         for i in range(len(old_wing)):
                    #             speed[i] = math.sqrt(
                    #                 ((new_wing[i][0] - old_wing[i][0])) ** 2 + ((new_wing[i][1] - old_wing[i][1])) ** 2) / fps
                    #         motion = sum(speed)
                    #         # print("total: " + str(motion))
                    #         # print("avg: " + str(motion/6))
                    # 
                    #     # msg = osc_message_builder.OscMessageBuilder(address="/energy" + str(counter))
                    #     # msg.add_arg(motion)
                    #     # msg = msg.build()
                    #     # client_Rip.send(msg)
                    #     # client_Nic.send(msg)
                    # 
                    #     window[0].pop(0)
                    #     window[1].pop(0)
                    #     window[2].pop(0)
                    #     window[3].pop(0)
                    #     window[4].pop(0)
                    #     window[5].pop(0)
                    # wing_history[track_id] = new_wing
                    # track_history[track_id] = window
                    # coordinate_history[track_id] = coordinate
                    counter += 1
                selfSock.sendto(str.encode(str(send_arr)), (UDP_IP_self, UDP_PORT_self))
        except:
            # print("no one detected")
            msg = osc_message_builder.OscMessageBuilder(address="/no_one")
            msg.add_arg("none")
            msg = msg.build()
            client_Nic.send(msg)
            client_Rip.send(msg)
            numP = 0
            numPBytes = numP.to_bytes(4, 'big')
            # selfSock.sendto(numPBytes, (UDP_IP_self, UDP_PORT_self))
            selfSock.sendto(str.encode("noone"), (UDP_IP_self, UDP_PORT_self))

            # new_wing2 = [[[np.float32(left_shoulder[0]), np.float32(left_shoulder[1])]], [[np.float32(right_shoulder[0]), np.float32(right_shoulder[1])]],
            #             [[np.float32(left_elbow[0]), np.float32(left_elbow[1])]], [[np.float32(right_elbow[0]), np.float32(right_elbow[1])]],
            #             [[np.float32(left_hand[0]), np.float32(left_hand[1])]], [[np.float32(right_hand[0]), np.float32(right_hand[1])]]]

                ### SPARSE OPTICAL FLOW IMPLEMENTATION
                # if p0 is None:
                #     old_frame = frame
                #     mask = np.zeros_like(old_frame)
                #     old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
                #     p0 = np.array(new_wing2)
                # else:
                #     p0 = np.array(new_wing2)
                #         # print("reset")
                #
                #     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #     p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                #     print(p0)
                #     print(p1)
                #     d = abs(p1-p0).reshape(-1, 2).max(-1)
                #     print(d)
                #     # print(p1)
                #     if p1 is not None:
                #         good_new = p1[st==1]
                #         good_old = p0[st==1]
                #     for i, (new, old) in enumerate(zip(good_new, good_old)):
                #         a, b = new.ravel()
                #         c, d = old.ravel()
                #         mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                #         frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
                #         img = cv2.add(frame, mask)
                #     cv2.imshow('frame', img)
                #     old_gray = frame_gray.copy()
                #     p0 = good_new.reshape(-1, 1, 2)


            # print(object.names)
            # if object.names[0] == "person":
                ### MEDIAPIPE SEGMENTATION
                # bounding_box = object.boxes.data.cpu()[0].tolist()
                # roi = frame[int(bounding_box[1]):int(bounding_box[3]),int(bounding_box[0]):int(bounding_box[2])]
                #
                # mp_results = pose.process(roi)
                # roi.flags.writeable = True
                # mp_drawing.draw_landmarks(roi, mp_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                # cv2.imshow('mp', roi)

                # keypoints = results[0].keypoints.xy.tolist()[0]
                # ann = Annotator(frame)
                # for i, kp in enumerate(keypoints):
                #     x = int(kp[0])
                #     y = int(kp[1])
                #     ann.text((x,y), str(i))


                    # print(dist)
                    # ann.text((50, 50), str(dist))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()

