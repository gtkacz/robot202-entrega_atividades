#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

naobateu = True

def scaneou(dado):
    global naobateu
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

    if dado.ranges[0] <= 1:
        naobateu = False
    elif dado.ranges[0] >= 1.02:
        naobateu = True




if __name__=="__main__":
    
    rospy.init_node("le_scan")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    
    v = 0.4
    w = 0.4
    
    frente = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
    afasta = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
    zero = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    while not rospy.is_shutdown():
        
        if naobateu == False:
            rospy.sleep(0.5)
            pub.publish(zero)
            rospy.sleep(0.02)
            pub.publish(afasta)
        elif naobateu == True:
            rospy.sleep(0.5)
            pub.publish(frente)


		
        
        
    
