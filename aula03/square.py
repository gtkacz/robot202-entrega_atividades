#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist, Vector3
import math

if __name__=="__main__":

    rospy.init_node("square")

    t0 = rospy.get_rostime()

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    
    n=0
    
    ang = math.radians(90.0)
    dist = 2
    
    w = 0.2
    v = 0.2
    
    zero = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    frente = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
    gira = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))

    sleep_reto = dist/v
    sleep_muda_direcao = ang/w

    while not rospy.is_shutdown():
        print("t0", t0)
        if t0.nsecs == 0:
            t0 = rospy.get_rostime()
            print("waiting for timer")
            continue        
        t1 = rospy.get_rostime()
        elapsed = (t1 - t0)
        print("Passaram ", elapsed.secs, " segundos")
        rospy.sleep(1.0)

        while n != 7:
            if n % 2 != 0:
                print("Girando")
                pub.publish(gira)
                rospy.sleep(sleep_muda_direcao)
                pub.publish(zero)
                rospy.sleep(0.1)
                n += 1

            if n % 2 == 0:
                print("Andando reto")
                pub.publish(frente)
                rospy.sleep(sleep_reto)
                pub.publish(zero)
                rospy.sleep(0.1)
                n += 1   
                
            rospy.sleep(1.0)

        print("Quadrado desenhado!")
            
        
