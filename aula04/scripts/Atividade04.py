#! /usr/bin/env python
# -*- config:utf-8 -*-

__author__ = ["Gabriel Mitelman Tkacz", "Rafael Malcervelli"]

import rospy
import tf
import math
import cv2
import time
import cor
from cormodule import identifica_cor
from geometry_msgs.msg import Twist, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from nav_msgs.msg import Odometry
import numpy as np

from tf import transformations

bridge = CvBridge()

x=None
y=None


diminuir_velocidade=False
chegou=False

def posicao_odometry(msg):
    global x
    global y

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    quat = msg.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    angs_degree = np.degrees(angulos_rad)


def callback(msg):
    global andar_frente
    global chegou
    lista_distancias=list(msg.ranges)
    contador_parar=0 

    for i in range(len(lista_distancias)):
        dist = lista_distancias[i]
        if (i < 21 or i > 339) and str(dist) != "inf":
            print(dist)

            if dist <= 0.25:
                contador_parar += 1
                if contador_parar >= 5:
                    print("PARA TUDO")
                    chegou = True

 
    contador_parar = 0
    andar_frente = True 

def roda_todo_frame(img):
    global cv_image
    global media
    global centro

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        media, centro, maior_area = identifica_cor(cv_image)
    except CvBridgeError as e:
        print("Problema no rospy: ", e)

cv_image = None
media, centro = [], []
area = 0.1
andar_frente = False
w=0.3
v=0.3

contador_deteccao_creeper = 1
        
if __name__=="__main__":

    rospy.init_node("Atividade04")
    topico_img = "/camera/rgb/image_raw/compressed"
    odom_sub = rospy.Subscriber("/odom", Odometry, posicao_odometry)
    recebe_img = rospy.Subscriber(topico_img, CompressedImage, roda_todo_frame, queue_size=4, buff_size=2**24)
    scanner    = rospy.Subscriber("/scan", LaserScan, callback)
    vel_saida  = rospy.Publisher("/cmd_vel", Twist, queue_size=3) 
    
    detectou = False
    vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
    
    is_centralizando = True
    contador_centralizacao = 0 
    girando_direita_t1, girando_direita_t2 = False, False
    while not rospy.is_shutdown():
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        if not chegou:
            if is_centralizando: 
                if len(media) != 0 and len(centro) != 0:
                    contador_deteccao_creeper += 1
                    if contador_deteccao_creeper >= 10:
                        detectou = True
    
                if detectou and media[0] > centro[0]:
                    girando_direita_t1 = True
                    # print(centro)
    
                elif detectou and media[0] < centro[0]:
                    girando_direita_t1 = False
                    # print(centro)
    
                if girando_direita_t1 != girando_direita_t2:
                    w /= -2
                    contador_centralizacao += 1
                    print(contador_centralizacao)
                    if contador_centralizacao >= 8:
                        is_centralizando = False
                        rospy.sleep(1)
    
                girando_direita_t2 = girando_direita_t1
                vel = Twist(Vector3(0,0,0), Vector3(0,0, w)) 
                vel_saida.publish(vel)
                rospy.sleep(0.1)
                #print("centralizando")
    
            else:
                # print('centralizou')
                if andar_frente:#andar_frente:
                    vel = Twist(Vector3(v,0,0), Vector3(0,0, 0)) 
                    vel_saida.publish(vel)
                    rospy.sleep(3)
    
                    is_centralizando = True
                    print('comeca a centralizar')
                    contador_centralizacao = 0 
                    w = .4
        else:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            vel_saida.publish(vel)
            rospy.sleep(0.2)