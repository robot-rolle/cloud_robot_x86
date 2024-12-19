#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Test for robot circle_move 
---------------------------
Moving around:
   q   w    e   r increase speed
   a        d
       s        z decline  speed
CTRL-C to quit
"""
#键值对应移动/转向方向
moveBindings = {
        'w':(1,0),
        'a':(0,1),
        'd':(0,-1),
        's':(-1,0),
        'q':(1,1),
        'e':(1,-1),
           }
#键值对应速度增量
speedBindings={
        'r':(1.1,1.1),
        'z':(0.9,0.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05) # time set 50ms 
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.1  #0.2m/s
turn = 0.1 # 0.1 rad/s

def time_auto_callback(event):
    _now = rospy.get_rostime()
    # rospy.loginfo("time_line time %i \r", _now.secs)
    # twist = Twist()
    # twist.linear.x = 0.0
    # twist.angular.z = 0.1

    
def ros_print(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)#获取键值初始化，读取终端相关属性
    
    rospy.init_node('ydrobot_teleop')#创建ROS节点
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)#创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'
    rospy.Timer(rospy.Duration(0.05),time_auto_callback) 
    time_count =0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    static_count = 0 #count of static set 
    
    x      = 0   #前进后退方向
    th     = 0   #转向/横向移动方向
    count  = 0   #键值不再范围计数
    target_speed = 0 #前进后退目标速度
    target_turn  = 0 #转向目标速度
    control_speed = 0 #前进后退实际控制速度
    control_turn  = 0 #转向实际控制速度
    control_state_auto = 0 #自动状态标识位 0 stop 1 正转 -1 反转

    try:
        print(msg)
        print(ros_print(speed,turn))
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
                control_state_auto =9
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0
                control_state_auto  =9
                print(ros_print(speed,turn))
            elif key == ' ' or key == 'h' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                control_state_auto = 1
                rospy.loginfo('-----circle-buffer----')
            elif key =='u' :
                x = 0
                th = 0
                control_speed =0
                control_turn = 0
                control_state_auto =2
                rospy.loginfo("-----line---buffer---")
            elif key == 'j':
                x=0
                th =0
                control_state_auto =3
                rospy.loginfo('------back---line----')
                control_speed =0
                control_turn =0
            elif key=='k':
                x=0
                th=0
                control_state_auto =4
                rospy.loginfo("-----back---circle---")
                control_speed=0
                control_turn=0            
            else:
                count = count + 1
                if count >=3: 
                    x = 0
                    th = 0
                    if control_state_auto !=9 & control_state_auto!=0:
                        rospy.loginfo("!=9")
                        control_state_auto=control_state_auto
                    elif control_state_auto==9:
                        rospy.loginfo(" 0")
                        control_state_auto = 0
                if (key == '\x03'):
                    break
            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th
            # 速度限位，防止速度增减过快
            if target_speed > control_speed:
                control_speed = target_speed 
            elif target_speed < control_speed:
                control_speed =  target_speed 
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # 创建并发布twist消息
            twist = Twist()
            twist_reset_=Twist()
            twist.linear.x = control_speed; 
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            if control_state_auto !=0 :
                if control_state_auto ==1 :
                    twist.linear.x = 0.0
                    twist.angular.z = 0.2
                elif control_state_auto ==2:
                    twist.linear.x = 0.2
                    twist.angular.z = 0
                elif control_state_auto ==3:
                    twist.linear.x =-0.2
                    twist.angular.z =0
                elif control_state_auto ==4:
                    twist.linear.x=0
                    twist.angular.z =-0.2
                elif control_state_auto ==9:
                    twist.linear.x = control_speed
                    twist.angular.z = control_turn 
                pub.publish(twist)
            else:
                if static_count==1:
                    twist_reset_.linear.x=0
                    twist_reset_.angular.z=0
                    static_count=0
                    pub.publish(twist_reset_)
                

    except Exception as e:
        rospy.loginfo("error-rr")
        rospy.logerror(e)
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)





