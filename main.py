#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# === ライブラリのインポート（画像処理・ROS通信・センサ処理） ===
import numpy as np 
import cv2, math    
import rospy, rospkg, time      
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge   
from xycar_msgs.msg import xycar_motor    
from sensor_msgs.msg import LaserScan
import signal   
import sys   
import os    
import time

# === PID制御：車線の中心からのズレを元にステアリングを計算 ===
def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error  
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error

    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error 
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

# Ctrl+Cで終了時の処理（ROSノードの強制停止）
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === グローバル変数（画像、センサデータ、制御用） ===
image = np.empty(shape=[0])
bridge = CvBridge()
motor = None  
img_ready = False  
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480 
ROI_ROW = HEIGHT - 150  
ROI_HEIGHT = HEIGHT - ROI_ROW   
L_ROW = 70
lidar_scan = None
SIMILARITY_THRESHOLD = 0.1
MAX_DETECTION_DISTANCE = 0.7
COUNT_LINE = 0
AVOID_COUNT = 0

# === カメラ画像の受信コールバック ===
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True  

# === 車の駆動指令送信関数 ===
def drive(Angle, Speed):
    global motor  
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed  
    motor.publish(motor_msg)

# === LiDARのデータ受信コールバック ===
def lidar_callback(data):
    global lidar_scan
    lidar_scan = data

# === 障害物の距離を左右・前方で測定 ===
def check_obstacles():
    global lidar_scan
    distances = {'front': 5.0, 'left': 5.0, 'right': 5.0}

    if lidar_scan is None:
        return distances

    ranges = lidar_scan.ranges
    front_zone = ranges[490:] + ranges[:14]
    valid_front = [r for r in front_zone if 0.0 < r < MAX_DETECTION_DISTANCE]
    if valid_front:
        distances['front'] = min(valid_front)

    right_zone = ranges[40:60]
    valid_right = [r for r in right_zone if 0.0 < r < MAX_DETECTION_DISTANCE]
    if valid_right:
        distances['right'] = min(valid_right)

    left_zone = ranges[440:460]
    valid_left = [r for r in left_zone if 0.0 < r < MAX_DETECTION_DISTANCE]
    if valid_left:
        distances['left'] = min(valid_left)

    return distances

# === 障害物回避時のS字走行戦略 ===
def avoid_s_curve(obstacle_distances):
    global AVOID_COUNT
    rospy.loginfo("Executing dynamic S-curve maneuver...")

    if obstacle_distances['right'] < MAX_DETECTION_DISTANCE and obstacle_distances['left'] == 5.0 :
        rospy.loginfo("Left side is clearer. Initiating LEFT-swerve.")
        return obstacle_distances['right']

    elif obstacle_distances['left'] < MAX_DETECTION_DISTANCE and obstacle_distances['right'] == 5.0 :
        rospy.loginfo("Right side is clearer. Initiating RIGHT-swerve.")
        return -obstacle_distances['left']

    elif obstacle_distances['left'] < MAX_DETECTION_DISTANCE and obstacle_distances['right'] < MAX_DETECTION_DISTANCE:
        rospy.loginfo("Left/Right both aren't clearer. Initiation TUNNEL-driving")
        return obstacle_distances['left'] - obstacle_distances['right']

    return None

# === 停止線（白線）の検出 ===
def detect_stop_line(x_midpoint):
    global image, L_ROW, ROI_ROW, WIDTH, HEIGHT, COUNT_LINE
    roi_width = 100
    roi_height = 15

    start_x = max(0, int(x_midpoint - roi_width / 2))
    end_x = min(WIDTH, int(x_midpoint + roi_width / 2))
    stop_line_y_center = ROI_ROW + L_ROW
    start_y = max(0, stop_line_y_center - roi_height / 2)
    end_y = min(HEIGHT, stop_line_y_center + roi_height / 2)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_img = cv2.threshold(gray,230,255,cv2.THRESH_BINARY)
    stop_line_roi = binary_img[start_y:end_y, start_x:end_x]

    total_pixels = stop_line_roi.size
    if total_pixels == 0:
        return False

    white_pixels = cv2.countNonZero(stop_line_roi)
    white_ratio = float(white_pixels) / total_pixels
    detection_threshold = 0.3

    if white_ratio > detection_threshold:
        COUNT_LINE += 1
        rospy.loginfo("White Line Detected! (%d time)", COUNT_LINE)
        return True
    else:
        return False

# === メイン処理：センサ統合→画像処理→制御指令 ===
def start():
    global image, img_ready
    global motor 
    prev_x_left, prev_x_right = 0, WIDTH 

    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)

    print("--Xycar--")
    rospy.sleep(2)

    # センサ準備完了待ち
    while not rospy.is_shutdown():
        if image.size != (WIDTH * HEIGHT * 3) or lidar_scan is None:
            time.sleep(0.1)
            continue
        else:
            print("Camera and LiDAR are ready.")
            break

    # === 走行ループ ===
    while not rospy.is_shutdown():
        while img_ready == False:
            continue

        # 画像の前処理（Canny, ROI抽出, Hough変換）
        img = image.copy()
        img_ready = False
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5,5),0)
        edge_img = cv2.Canny(np.uint8(blur_gray),30,60)
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        offset_max = 100

        # 車線検出ができない場合はLiDARによる補正
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 80, 80, 10)
        if all_lines is None:
            obs_dist = check_obstacles()
            if obs_dist['left'] < 1.5 and obs_dist['right'] < 1.5:
                rospy.loginfo("Tunnel Started")
                tmp_midpoint = obs_dist['left'] - obs_dist['right']
                offset = int(tmp_midpoint * 200)
                offset = max(min(offset, offset_max), -offset_max)
                x_midpoint = 320 - offset
                angle = PID(x_midpoint, 0.80, 0.0003, 0.15)
                drive(angle, 20)
                continue
            else:
                drive(0, 20)
                continue

        # ここで車線を左右に分類し、中心点を計算し制御角度に変換（省略）
        # ※ 本体コードにて処理中...

        # 停止線検出
        if detect_stop_line(x_midpoint):
            if COUNT_LINE == 1:
                rospy.loginfo("停止線検出：即時停止")
                drive(0, 0)
                rospy.sleep(5)
            elif COUNT_LINE == 2:
                rospy.loginfo("2本目の停止線：徐行後停止")
                drive(-20, 5)
                rospy.sleep(4)
                break

        # 障害物がある場合、S字回避処理を実行
        obs_dist = check_obstacles()
        if obs_dist['left'] != 5.0 or obs_dist['right'] != 5.0:
            rospy.sleep(0.1)
            obs_dist = check_obstacles()
            tmp_midpoint = avoid_s_curve(obs_dist)
            if tmp_midpoint is not None:
                offset = int(tmp_midpoint * 200)
                offset = max(min(offset, offset_max), -offset_max)
                x_midpoint = 320 - offset

        # 最終的なPID制御出力を使って駆動
        angle = PID(x_midpoint, 0.60, 0.0003, 0.10)
        speed = 5
        drive(angle, speed)

        # ユーザーが'q'を押すと終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User Exit")
            break

# === 実行エントリーポイント ===
if __name__ == '__main__':
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()
    start()

