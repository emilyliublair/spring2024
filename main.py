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
UDP_PORT_self = 5006
#
# # sock.sendto(b"hi", (UDP_IP, UDP_PORT))
client_Nic = udp_client.UDPClient(UDP_IP_Nic, UDP_PORT_Nic)
client_Rip = udp_client.UDPClient(UDP_IP_Rip, UDP_PORT_Rip)
selfSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# client_self = udp_client.UDPClient(UDP_IP_self, UDP_PORT_self)

model = YOLO('yolov8n-pose.pt')
# mp_pose = mp.solutions.pose
# mp_drawing = mp.solutions.drawing_utils

video_path = "./vid.mp4"
cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(2)

# velocity calculation
# old_wing = None
old_pos_un = None
speed = [0, 0, 0, 0, 0, 0]
prev_time_frame = 0
new_time_frame = 0
# window = [[], [], [], [], [], []]

# track
# track_history = defaultdict(lambda: [[], [], [], [], [], []])
# wing_history = defaultdict(lambda: [])
# coordinate_history = defaultdict(lambda: [])
coordinate_history_un = defaultdict(lambda: [])

# pts1 = np.float32([[0, 416], [106, 164], [508, 130], [640, 318]])
# pts2 = np.float32([[0, height], [0, 0], [width, 0], [width, height]])
# matrix = cv2.getPerspectiveTransform(pts1, pts2)

p1 = (114, 79)
p2 = (590, 226)
line_slope = -.2938

vid_path = "./vid2.mp4"
# cap = cv2.VideoCapture(vid_path)

# with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
while cap.isOpened():
    success, frame = cap.read()
    if success:
        ### reset tracking to conserve space
        # if len(track_history) > 50:
        #     print("reset")
        #     # track_history = defaultdict(lambda: [[], [], [], [], [], []])
        #     # wing_history = defaultdict(lambda: [])
        #     coordinate_history = defaultdict(lambda: [])
        #     coordinate_history_un = defaultdict(lambda: [])
        #     old_wing = None
        #     speed = [0, 0, 0, 0, 0, 0]
        #     prev_time_frame = 0
        #     new_time_frame = 0
        #     window = [[], [], [], [], [], []]

        results = model.track(frame, save=False, persist=True, conf=0.65)
        annotated_frame = results[0].plot()

        # fps
        new_time_frame = time.time()
        fps = int(1 / (new_time_frame - prev_time_frame))
        prev_time_frame = new_time_frame
        cv2.putText(annotated_frame, "fps: " + str(fps), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)


        # cv2.circle(annotated_frame, ellipse_center, radius=0, color=(0, 255, 0), thickness=-1)

        cv2.imshow("YOLOv8 Tracking", annotated_frame)
        # result = cv2.warpPerspective(annotated_frame, matrix, (width, height))
        # cv2.imshow("scaled", result)

        try:
            boxes = results[0].boxes.xywh.cpu()
            keypoints = results[0].keypoints
            track_ids = results[0].boxes.id.int().cpu().tolist()
            counter = 0

            send_arr = []

            for box, kps, track_id in zip(boxes, keypoints, track_ids):
                send = True

                # window = track_history[track_id]
                # old_wing = wing_history[track_id]
                # coordinate = coordinate_history[track_id]
                coordinate_un = coordinate_history_un[track_id]

                kp = kps.xy.tolist()[0]
                kp_n = kps.xyn.tolist()[0]

                # left_shoulder = kp[5]
                # left_shoulder.append(1)
                # left_shoulder = np.matmul(left_shoulder, matrix)
                # left_elbow = kp[7]
                # left_elbow.append(1)
                # left_elbow = np.matmul(left_elbow, matrix)
                # left_hand = kp[9]
                # left_hand.append(1)
                # left_hand = np.matmul(left_hand, matrix)
                # right_shoulder = kp[6]
                # right_shoulder.append(1)
                # right_shoulder = np.matmul(right_shoulder, matrix)
                # right_elbow = kp[8]
                # right_elbow.append(1)
                # right_elbow = np.matmul(right_elbow, matrix)
                # right_hand = kp[10]
                # right_hand.append(1)
                # right_hand = np.matmul(right_shoulder, matrix)

                # left_foot = kp_n[15]
                # right_foot = kp_n[16]
                left_foot_un = kp[15]
                right_foot_un = kp[16]
                # left_foot_scaled = left_foot_un
                # left_foot_scaled.append(1)
                # left_foot_scaled = np.matmul(left_foot_scaled, matrix)
                # right_foot_scaled = right_foot_un
                # right_foot_scaled.append(1)
                # right_foot_scaled = np.matmul(right_foot_scaled, matrix)

                # window[0].append(left_shoulder)
                # window[1].append(right_shoulder)
                # window[2].append(left_elbow)
                # window[3].append(right_elbow)
                # window[4].append(left_hand)
                # window[5].append(right_hand)

                # pos_scaled = (np.array([left_foot_scaled[0], left_foot_scaled[1]]) + np.array(
                #     [right_foot_scaled[0], right_foot_scaled[1]])) / 2
                pos_un = (np.array(left_foot_un) + np.array(right_foot_un)) / 2
                coordinate_un.append(pos_un)
                print(pos_un)

                if len(coordinate_un) >= 3:
                    send_coordinate_un = np.mean(np.array(coordinate_un), axis=0)
                    if old_pos_un is not None:
                        vel = (np.subtract(send_coordinate_un, old_pos_un)) * fps
                        vel_msg_un = str(vel[0]) + ", " + str(vel[1])
                        coordinate_msg_un = str(send_coordinate_un[0]) + ", " + str(send_coordinate_un[1])
                        send_msg = coordinate_msg_un + " / " + vel_msg_un
                        print(send_msg)
                        send_arr.append(send_msg)

                    old_pos_un = send_coordinate_un
                    coordinate_un.pop(0)
                counter += 1
            print(send_arr)
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
            selfSock.sendto(str.encode("noone"), (UDP_IP_self, UDP_PORT_self))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
