#!/usr/bin/env python
# ROBOT CODE

import rospy
from robot_sim.msg import Encoder
from geometry_msgs.msg import Twist

t = 20
T = 0.2
L = 0.287
r = 0.033
dt = 0.1
pi = 3.14159

def updateW(W1_prev, W2_prev, W1_tar, W2_tar):
    beta = dt / (dt + T)
    W1n = beta * W1_prev + (1-beta) * W1_tar
    W2n = beta * W2_prev + (1-beta) * W2_tar
    return (W1n, W2n)
    
def wheelVel(V, Omega):
    R = V / Omega
    l = R - L/2
    W_left = Omega * l/r
    l = R + L/2
    W_right = Omega * l/r
    return W_left, W_right

def velToEncoder (W1, W2): 
    n1 = W1 * dt / (2 * pi)
    n2 = W2 * dt / (2 * pi)
    e1 = int(n1 * 4096)
    e2 = int(n2 * 4096)
    return (e1, e2)


def sendToServ(W1_tar, W2_tar):
    pub = rospy.Publisher('RobotToServer', Encoder, queue_size=10)

    W1_prev = W2_prev = 0
    message_id = 0
    
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while now < rospy.Duration.from_sec(20):
        W1_new, W2_new = updateW(W1_prev, W2_prev, W1_tar, W2_tar)
        e1, e2 = velToEncoder(W1_new, W2_new)

        msg = Encoder()
        msg.leftWheel = e1
        msg.rightWheel = e2
        
        frame_id = "Measurement #" + str(message_id)

        msg.header.stamp = rospy.Time.now()
        msg.header.seq = message_id
        msg.header.frame_id = frame_id
        
        try: 
            pub.publish(msg)
        except rospy.ROSInterruptException:
            continue
            
        log = str(frame_id) + ": Send to serv left: " + str(e1)
        log += " and right: " + str(e2) + "  Encoders.\nTime: "
        log += str(msg.header.stamp)
            
        rospy.loginfo(log)

        message_id += 1
        W1_prev, W2_prev = W1_new, W2_new
        rate.sleep()
        
def robotCallback(data):
    V = data.linear.x
    Omega = data.angular.z
    
    log = str(rospy.get_caller_id()) + ' Get from serv V: ' + str(V)
    log += '(m/s) and Omega ' + str(Omega) + '(rad/s). Time: '
    log += str(rospy.Time.now())
    
    rospy.loginfo(log)
  
    W1_tar, W2_tar = wheelVel(V, Omega)
    sendToServ(W1_tar, W2_tar)
    
    
def listenServ():
    rospy.init_node('robot', anonymous = True)
    rospy.Subscriber('ServerToRobot', Twist, robotCallback)
    rospy.spin()
    
if __name__=="__main__":
    listenServ()
