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
client_Nic = udp_client.UDPClient(UDP_IP_Nic, UDP_PORT_Nic)
client_Rip = udp_client.UDPClient(UDP_IP_Rip, UDP_PORT_Rip)
selfSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

model = YOLO('yolov8n-pose.pt')

video_path = "./vid.mp4"
cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(2)

# velocity calculation
old_pos_un = None
prev_time_frame = 0
new_time_frame = 0

# track
coordinate_history_un = defaultdict(lambda: [])

z2p1 = (300, 227)
z2p2 = (300, 480)
z2p3 = (0, 331)
z2p4 = (236, 195)

p1 = (61, 299)
p2 = (300, 397)

z3p1 = (401, 222)
z3p2 = (636, 428)
z3_slope = 0.8766

middlearmzonep1 = (250, 0)
middlearmzonep2 = (250, 480)
middlearmzonep3 = (470, 0)
middlearmzonep4 = (470, 480)

vid_path = "./vid2.mp4"
# cap = cv2.VideoCapture(vid_path)

# with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
while cap.isOpened():
    success, frame = cap.read()
    if success:
        ### reset tracking to conserve space
        if len(coordinate_history_un) > 50:
            print("reset")
            coordinate_history_un = defaultdict(lambda: [])
            old_pos_un = None
            prev_time_frame = 0
            new_time_frame = 0

        results = model.track(frame, save=False, persist=True, conf=0.65)
        annotated_frame = results[0].plot()

        # fps
        new_time_frame = time.time()
        fps = int(1 / (new_time_frame - prev_time_frame))
        prev_time_frame = new_time_frame
        cv2.putText(annotated_frame, "fps: " + str(fps), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        
        cv2.line(annotated_frame, z2p1, z2p2, (255, 0, 0), 2)
        cv2.line(annotated_frame, z2p3, z2p4, (255, 0, 0), 2)
        cv2.line(annotated_frame, p1, p2, (0, 255, 0), 2)

        cv2.line(annotated_frame, middlearmzonep1, middlearmzonep2, (0, 0, 255), 2)
        cv2.line(annotated_frame, middlearmzonep3, middlearmzonep4, (0, 0, 255), 2)
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

        try:
            boxes = results[0].boxes.xywh.cpu()
            keypoints = results[0].keypoints
            track_ids = results[0].boxes.id.int().cpu().tolist()
            counter = 0

            send_arr = []

            for box, kps, track_id in zip(boxes, keypoints, track_ids):
                send = True

                coordinate_un = coordinate_history_un[track_id]

                kp = kps.xy.tolist()[0]
                kp_n = kps.xyn.tolist()[0]
                
                left_foot_un = kp[15]
                right_foot_un = kp[16]

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
