#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import cv2
import numpy as np
import time

# 获取键盘按键
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 降低延迟，提升响应性
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 恢复终端设置
    return key

# 摄像头初始化
def init_camera(frame_width, frame_height):
    cap = cv2.VideoCapture(0)
    time.sleep(0.5)  # 等待0.5秒，保证摄像头正常启动
    cap.set(3, frame_width)  # 设置图像宽度
    cap.set(4, frame_height)  # 设置图像高度
    cap.set(10, 30)  # 设置视频帧率
    cap.set(cv2.CAP_PROP_EXPOSURE, 0)  # 初始曝光值
    if not cap.isOpened():
        raise Exception("无法打开摄像头")
    return cap

# 调用曝光调整
def adjust_exposure(cap, avg_brightness):
    if avg_brightness > 200:  # 需要自己调整正常的范围上限
        current_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        cap.set(cv2.CAP_PROP_EXPOSURE, max(current_exposure - 1, -10))  # 降低曝光
    elif avg_brightness < 100:  #  需要自己调整正常的范围下限
        current_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        cap.set(cv2.CAP_PROP_EXPOSURE, min(current_exposure + 1, 0))  # 增加曝光
    else:  # 不进行调整
        return

# 检测颜色区域，由于只需要识别距离较近的锥桶，所以区域面积如果大于4000，那么则进行判断
def detect_color_areas(mask, area_threshold=4000):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 查找轮廓
    count = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > area_threshold:  # 根据设定阈值筛选颜色区域
            x, y, w, h = cv2.boundingRect(cnt)
            if w * h <= 80 * 80:  # 判断是否满足一定大小，如果超出80*80即6400那么存取，否则过滤
                count += 1
    return count

# 控制转向函数
def turn(twist, direction, pub, cap, lcnt, rcnt):
    twist.linear.x = 1545
    twist.angular.z = 165 if direction == "left" else 15  # 设置左转或右转的角度
    pub.publish(twist)

    # 由于发布是瞬间的，所以小车可能会出现接受下一个指令时出现重置导致又接收到直行命令，如果
    # 使用time.sleep(time)指令，可能存在不连贯的情况，所以使用while循环持续读取视频帧。
    # 如果在视频帧中读取到直行的条件后，那么则发布直行指令，否则一直进行转向操作
    while cap.isOpened():  # 循环读取摄像头帧以调整方向
        # 为防止小车进程卡主无法停车而向前冲刺，将键盘输入放在此处用于读取进行停车操作
        key = getKey()  # 获取键盘按键输入
        if key == '\x03':  # Ctrl + C 退出
            break
        ret, frame = cap.read()
        if not ret:
            break
        if lcnt > 0 and rcnt > 0:  # 如果红蓝同时检测到，恢复直行
            twist.linear.x = 1550
            twist.angular.z = 90
            pub.publish(twist)
            time.sleep(0.5)  # 延迟以确保转向平稳
            break

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)  # 保存终端设置
    rospy.init_node('racecar_teleop')  # 初始化ROS节点
    pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)  # 创建话题发布者

    # 参数初始化
    speed_mid = 1545
    turn_mid = 90
    control_speed = speed_mid  # 当前速度
    control_turn = turn_mid  # 当前转向
    run = 1  # 设置为运行状态

    # 摄像头参数
    frame_width = 640
    frame_height = 480
    cap = init_camera(frame_width, frame_height)  # 初始化摄像头

    try:
        while True:
            key = getKey()  # 获取键盘按键输入
            twist = Twist()  # 初始化Twist消息

            ret, frame = cap.read()  # 从摄像头获取帧
            if not ret:
                break  # 如果读取失败则退出循环

            # 计算平均亮度
            avg_brightness = cv2.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[0]
            adjust_exposure(cap, avg_brightness)  # 调整曝光

            # 转换图像到HSV颜色空间
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 红色和蓝色的HSV颜色范围，这里需要根据实际情况做一定的调整
            min_red = np.array([156, 43, 46])
            max_red = np.array([180, 255, 255])
            min_blue = np.array([100, 43, 46])
            max_blue = np.array([130, 255, 255])

            # 创建红色和蓝色掩膜
            mask_red = cv2.inRange(img_hsv, min_red, max_red)
            mask_blue = cv2.inRange(img_hsv, min_blue, max_blue)

            # 检测红色和蓝色区域
            rcnt = detect_color_areas(mask_red)  # 检测红色
            lcnt = detect_color_areas(mask_blue)  # 检测蓝色

            # 根据检测到的颜色数量进行控制
            if lcnt > 0 and rcnt > 0:
                # 如果红蓝同时检测到，直行
                twist.linear.x = 1550
                twist.angular.z = 90
                pub.publish(twist)
            elif rcnt < 2 and lcnt > 0:
                # 左转逻辑
                turn(twist, "left", pub, cap, lcnt, rcnt)
            elif lcnt < 2 and rcnt > 0:
                # 右转逻辑
                turn(twist, "right", pub, cap, lcnt, rcnt)

            # 控制车的运动状态
            if run == 1:
                twist.linear.x = control_speed
                twist.angular.z = control_turn
                pub.publish(twist)
            else:
                twist.linear.x = speed_mid
                twist.angular.z = turn_mid
                pub.publish(twist)

            if key == '\x03':  # Ctrl + C 退出
                break

    except Exception as e:
        print(f"Error: {str(e)}")

    finally:
        # 停止时设置小车的速度为默认停车值和转向为中间值
        twist.linear.x = 1500
        twist.angular.z = 90
        pub.publish(twist)

        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 恢复终端设置
