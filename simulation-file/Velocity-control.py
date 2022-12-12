#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import time
from math import cos, sin

import numpy as np
import rospy
import tf
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, Twist

from gazebo_msgs.msg import ModelStates

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
# ---------- 変えてみよう ---------- #

A = 0.5             # ゲイン
T = 1              # 入力する間隔
x = -1               # 所望のx座標[m]
y = 0               # 所望のy座標[m]

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
# ---------- 初期位置の場合 ---------- #

kp = 0.5, T = 5 振動(一番良さそう)

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
# ---------- 使い方・考察 ---------- #

teleopなどプログラムを動かすターミナルが時間経過で機能しなくなることがあるので注意する
速度制限をかけないと謎の回転方向の速度が生じてしまう

① 1つ目のターミナルで「roscore」を立てる
② 2つ目のターミナルで「roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch」でgazeboを開く
③ 3つ目のターミナルで「rosrun フォルダ名 Velocity-control.py」で本プログラムを動作させる

④ 「Ctrl + Shift + R」で初期の位置に戻すことができる

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
# ---------- 詳細 ---------- #

製作日：2022年8月17日
目的：gazeboシミュレーションによる定置制御において、速度入力の間隔を変えるとどうなるのかを確かめるプログラム

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

def main():

    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

    world_frame = "odom"
    turtle_frame = "base_footprint"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    time.sleep(1)

    speed = Twist()

    print('Experiment Start')

    l = 0.1                      # 前方位置[m]
    flag = True
    Ts = time.time()
    Tsample = 0.01
    ts = -T
    state = 1

    while True:
        t = time.time()-Ts
        roopTs = time.time()
        # データを取得する
        try:
            record = tfBuffer.lookup_transform(world_frame, turtle_frame, rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            continue 

        # クオータニオン(四元数)の取得
        Quaternion = (record.transform.rotation.x,record.transform.rotation.y,record.transform.rotation.z,record.transform.rotation.w)
        # オイラー角の取得
        RPY = tf_conversions.transformations.euler_from_quaternion(Quaternion)  

        xt = record.transform.translation.x
        yt = record.transform.translation.y

        thetat = RPY[2]            # yaw角
        xc = xt + l * cos(thetat)     # 前方位置を含めた座標位置
        yc = yt + l * sin(thetat)

        # 目標位置
        goal = np.array([[x], [y], [0]])   # (x,y,z)

        if abs(xc) > 10:
            flag = False
            print("NoWay")
            vx = 0
            speed.linear.x = vx
            vy = 0
            speed.angular.z = vy

        # 速度入力
        if time.time() - ts > T:
            if flag:
                # vx = (kp * T * (goal[0, 0] - xc)) * cos(thetat)
                # vy = 0
                vx = (A * T * (goal[0, 0] - xc)) * cos(thetat) + (A * T * (goal[1, 0])) * sin(thetat)
                if vx > 0.22:
                    vx = 0.21
                if vx < -0.22:
                    vx = -0.21
                # if vx > 0.41:
                #     vx = 0.40
                # if vx < -0.41:
                #     vx = -0.40
                speed.linear.x = vx
                print(vx)
            else:
                print("loading")
            ts = time.time()

        vy = (A * T * (goal[0, 0] - yc)) * sin(thetat) + (A * T * (goal[1, 0])) * cos(thetat) * (1/l)
        if vy > 2.84:
            vy = 2.83
        if vy < -2.84:
            vy = -2.83
        speed.angular.z = vy
        pub.publish(speed)
        roopT = time.time()-roopTs
        if roopT < Tsample:
            time.sleep(Tsample - roopT)
            # print("During the Experiment(state:", end="")
            # print(state, end="")
            # print(")")
            # state += 1

        # if KeyboardInterrupt:
        #     vx = 0
        #     speed.linear.x = vx
            
        if t > 60:
            break

if __name__ == '__main__':
    rospy.init_node('gazebo_velocity_stability')
    main()
