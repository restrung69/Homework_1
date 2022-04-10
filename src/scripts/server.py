#!/usr/bin/env python
# SERVER CODE

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from robot_sim.msg import Encoder

Encoder_data = [] # e1, e2
Messages = [] # index
Measures = [] # time of measures
Wheels_speed = []
Robot_speed = []
Robot_ang_spd = []
Robot_angle = [0]
Coordinates = [[0,0]]

t = 20
T = 0.2
L = 0.287
r = 0.033
dt = 0.1
pi = 3.14159

def position():
    temp = (Robot_speed[-1] * np.cos(Robot_angle[-1]) + Robot_speed[-2] * np.cos(Robot_angle[-2])) * dt/2
    x = temp + Coordinates[-1][0]

    temp = (Robot_speed[-1] * np.sin(Robot_angle[-1]) + Robot_speed[-2] * np.sin(Robot_angle[-2])) * dt/2
    y = temp + Coordinates[-1][1]
    
    theta = Robot_angle[-1]
    publishPosition(x, y, theta) 
    Coordinates.append([x,y])
    
    
def upd_Robot_ang():
    temp = (Robot_ang_spd[-1] + Robot_ang_spd[-2]) * dt/2
    Robot_angle.append(temp + Robot_angle[-1])
    
    
def upd_Robot_ang_spd():
    W1 = Wheels_speed[-1][0]
    W2 = Wheels_speed[-1][1]
    Omega = r/L * (W2 - W1)
    Robot_ang_spd.append(Omega)


def upd_Robot_speed():
    w1 = Wheels_speed[-1][0]
    w2 = Wheels_speed[-1][1]
    V = r/2 * (w1 + w2)
    Robot_speed.append(V)


def upd_Wheel_speed():
    e1 = Encoder_data[-1][0]
    e2 = Encoder_data[-1][1]
    w1 = e1 * 2 * pi / (4096 * dt)
    w2 = e2 * 2 * pi / (4096 * dt)
    Wheels_speed.append([w1,w2])

def Perfect_Trajectory():
    x_ideal = [0]
    y_ideal = [0]
    ideal_V = Robot_speed[-1]
    ideal_Omega = Robot_ang_spd[-1]
    for i in range(len(Measures) - 1):
        ideal_theta = ideal_Omega * dt * i 
        x = ideal_V * np.cos(ideal_theta) * dt
        x_ideal.append(x + x_ideal[-1])
        y = ideal_V * np.sin(ideal_theta) * dt
        y_ideal.append(y + y_ideal[-1])
        
    plt.plot(x_ideal, y_ideal)
    plt.title("ideal")
    plt.show()
        
              
def publishPosition(x, y, theta):
    pub = rospy.Publisher('Printing', Pose2D, queue_size = 10)
    
    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = theta
    
    pub.publish(pose)
    log = 'Printing coordinates x = ' + str(x)
    log +=  '(m), y = ' + str(y)
    log += '(m), theta = ' + str(theta) + '(rad).\nTime: '
    log += str(rospy.Time.now())
    
    rospy.loginfo(log)

def servCallback(data):
    e1 = data.leftWheel
    e2 = data.rightWheel
    sending_time = data.header.stamp
    message_index = data.header.seq
    message_id = data.header.frame_id
    
    log = rospy.get_caller_id() + ' ' + str(message_id)
    log += ': Get from robot left: ' + str(e1) + ', right: ' + str(e2)
    log += ' Encoders.\nTime:' + str(sending_time) + '. Receiving time: '
    log += str(rospy.Time.now())

    rospy.loginfo(log) 
 
    Encoder_data.append([e1,e2])
    Measures.append(sending_time)
    Messages.append(message_index)

    upd_Wheel_speed()
    upd_Robot_speed()
    Robot_ang_spd()
    if (len(Measures) > 1):
        upd_Robot_ang()
        position()

        
def listenRobot():
    rospy.Subscriber('RobotToServer', Encoder, servCallback)
    rospy.spin()


def sendToRobot(V, Omega):
    pub = rospy.Publisher('ServerToRobot', Twist, queue_size = 10)

    vel = Twist()
    vel.linear.x = V
    vel.angular.z = Omega
    
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while rospy.Time.now() < now + rospy.Duration.from_sec(0.2):
        pub.publish(vel)
        rate.sleep()
        

    log = "Send to robot V: " + str(V) + "(m/s), Omega: "
    log += str(Omega) + "(rad/s). Time: " + str(rospy.Time.now())
    
    rospy.loginfo(log)
     
    
if __name__=="__main__":
    rospy.init_node('server', anonymous=True)
    
    sendToRobot(1, 1)
    listenRobot()
    
    x = [i[0] for i in Coordinates]
    y = [i[1] for i in Coordinates]
    
    plt.plot(x, y)
    plt.title("real")
    plt.show()
    Perfect_Trajectory()