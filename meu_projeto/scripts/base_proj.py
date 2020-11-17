#! /usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function, division

__author__ = ["Gabriel Mitelman Tkacz", "Rafael Selcali Malcervelli", "Enrico Venturini Costa"]

import numpy as np
import numpy
import tf, math, cv2, time, rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float64
import cv_bridge
import auxiliar
import aruco
import pandas as pd

bridge=CvBridge()
cv_image=None
avg=[]
centro=[]
atraso=1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay=False
rospack=rospkg.RosPack() 
tfl = 0
tf_buffer = tf2_ros.Buffer()

avg=[]
center=[]
isTrack=True
isCreeper=False
chegou=False

x=0
y=0
z=0
theta=0
v=0.2
w=0.2
dist=0.15
MobileNetResults=[]

clawIsClosed=None
clawIsOpen=None
armIsRetracted=None
armIsStretched=None
armIsRaised=None
ID=None
og=None
match=None

creeperIDed=False
mode=False
state=False
flag=True

def posicao_odometry(data):
    global x
    global y
    global theta

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    pos = [quat.x, quat.y, quat.z, quat.w]
    angs_rad = transformations.euler_from_quaternion(pos)
    angs_degree = np.degrees(angs_rad)
    theta = angs_rad[2]

def scaneou(img):
    global chegou
    global dist
    if img.ranges[0] <= dist:
        chegou=True
    else:
        chegou=False
    
def go_to(x2, y2, pub):
    delta=math.atan2(y2-y, x2-x)
    ang=delta-theta

    sleep=ang/w
    pub.publish(velocityTwistCounterClockwise)
    rospy.sleep(sleep)

    sleep=0.75/v
    pub.publish(velocityTrackForward)
    rospy.sleep(sleep)

    
def halfTurn(pub):
    ang=math.pi

    sleepLinear=(0.75/v)
    sleepAngular=(ang/w)

    pub.publish(velocityTwistClockwise)
    rospy.sleep(sleepAngular)

    pub.publish(velocityTrackForward)
    rospy.sleep(sleepLinear)


def direcao_robo_pista(pub):
    global state

    if state == 'forward':
        pub.publish(velocityTrackForward)

    if state == 'left':
        pub.publish(velocityTrackLeft)

    if state == 'right':
        pub.publish(velocityTrackRight)

    if state == 'end':
        halfTurn(pub)


def controla_garra():
    global mode
    global state
    global creeperIDed

    if state == 'levanta garra':
        # Depois de estar a uma distancia pre-estabelecida, o creeper levanta o braco e abre a garra
        # Feito isso, o mode muda para aproxima do creeper
        braco_publisher.publish(armIsStretched)
        garra_publisher.publish(clawIsOpen)
        rospy.sleep(0.2)
        state = 'aproxima do creeper'

    elif state == 'aproxima do creeper':
        velocidade_saida.publish(velocityParked)
        rospy.sleep(0.2)
        tempo_aproxima = (dist - 0.225)/0.05
        velocidade_saida.publish(velocityApproachCreeper)
        rospy.sleep(tempo_aproxima)
        # Anda um distancia nao_bateu lentamente 
        #feito isso, muda pra agarra creeper
        state = 'agarra creeper'
    
    elif state == 'agarra creeper':
        garra_publisher.publish(clawIsClosed)
        rospy.sleep(0.1)
        braco_publisher.publish(armIsRaised)
        rospy.sleep(0.1)
        # Fecha a garra, levanta o braco, muda state pra retorna pra pista
        state ='retorna pista'        
        
    elif state == 'retorna pista':
        go_to(ponto[0], ponto[1], velocidade_saida)
        creeperIDed = True
        mode = 'procurando pista'
        state = ""

def roda_todo_frame(img):
    global cv_image
    global avg
    global center
    global MobileNetResults
    global isTrack
    global avgCreeper
    global cmCreeper
    global isCreeper
    global og

    now = rospy.get_rostime()
    imgtime = img.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        img1 = temp_image.copy()
        centro, saida_net, MobileNetResults =  auxiliar.processa(temp_image)        
        cv_image = saida_net.copy()
        avg, center, isTrack =  auxiliar.identifica_pista(temp_image, "blue")
        avgCreeper, cmCreeper, isCreeper =  creeper.isCreeper(temp_image, goal[0])
        
        # Identificação do ID
        ID = aruco.idTag(img1)
        og = ID[0][0]

        cv2.imshow("cv_image", temp_image)
        cv2.imshow("img1", img1)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)


if __name__=="main_":
    rospy.init_node("Projeto") 
    topico_img="/camera/image/compressed"
    odom_sub = rospy.Subscriber("/odom", Odometry, posicao_odometry)
    recebe_img=rospy.Subscriber(topico_img, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    scanner = rospy.Subscriber("/scan", LaserScan, scaneou)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    tfl = tf2_ros.TransformListener(tf_buffer)
    braco_publisher = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=1)
    garra_publisher = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=1)

    goal=("blue", 23, "dog")

    clawIsClosed=0
    clawIsOpen=-1
    armIsRetracted = -1.5
    armIsStretched = 0
    armIsRaised = 1.5
    ID=None
    og=0
    match=0

    velocityParked = Twist(Vector3(0,0,0), Vector3(0,0,0))
    velocityTrackForward = Twist(Vector3(v,0,0), Vector3(0,0,0))
    velocityTrackRight = Twist(Vector3(v,0,0), Vector3(0,0,-w))
    velocityTrackLeft = Twist(Vector3(v,0,0), Vector3(0,0,w))
    velocityCreeperRight = Twist(Vector3(v/3,0,0), Vector3(0,0,-w/3))
    velocityCreeperLeft = Twist(Vector3(v/3,0,0), Vector3(0,0,w/3))
    velocityApproachCreeper = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
    velocityTwistCounterClockwise = Twist(Vector3(0,0,0), Vector3(0,0,w))
    velocityTwistClockwise = Twist(Vector3(0,0,0), Vector3(0,0,-w))

    try:
        rospy.sleep(2.0)
        mode = 'procurando pista'
        while not rospy.is_shutdown():
            if mode == 'procurando pista':
                # Se eu nao identifico creeper eu procuro contorno
                if not isCreeper: 
                    while not isTrack:
                        velocidade_saida.publish(velocityTwistCounterClockwise)
                        rospy.sleep(0.01)
                    # Se eu identificar o contorno mudo meu mode e passo a seguir a pista    
                    mode = 'segue pista'

                # Se eu acho o creeper mudo o mode
                if isCreeper and creeperIDed==False: 
                    mode = 'creeper a la vista'

            if mode == 'segue pista':

                #precisa colocar a centralizacao da pista e o atualizar o state de acordo
                #precisa atualizar mode caso veja um creeper
                #precisa atualizar mode caso nao veja mais a pista
                try:
                    if not chegou:
                        print("Encontrei pista")
                        if (avg[0] > center[0]):
                            state = 'right'
                        elif (avg[0] < center[0]):
                            state = 'left'
                        else:
                            state = 'forward'

                    else: #quando ve o final da pista vira 180 e anda 0.8 pra frente
                        print("Encontrei algum obstáculo")
                        state = 'end'
                except:
                    pass

                
                # Mudança de mode
                if not isTrack:
                    mode = 'procurando pista'

                if isCreeper and creeperIDed==False:
                    mode = 'creeper a la vista'  

                # Chama a função direção_robo_pista, que seta a velocidade do robô com base nos substados
                direcao_robo_pista(velocidade_saida)
                rospy.sleep(0.01)  

            # Creeper foi creeperIDed
            if mode == 'creeper a la vista':

                if flag:
                    ponto = (x,y)
                    print("Ponto gravado:", ponto)
                    flag = False
                    print('Hasta la vista babeeeeee! Ponto: ', ponto) 

                if not chegou: 
                    state = 'segue creeper'
                    print('rosa here I goooo')
        
                else:
                    # Se for verdadeiro (match ID desejado com ID creeperIDed):
                    if match > 200:
                        mode = 'pega creeper'
                        state = 'levanta garra' #levanto a garra assim que mudo o mode
                    

                    # Se for falso, volta para a pista:
                    else:
                        print("Entrei na go to")
                        go_to(ponto[0], ponto[1], velocidade_saida)
                        mode = 'procurando pista'
                        match = 0

                
                if state == 'segue creeper':
                    print("matchs:", match)
                    
                    if og == goal[1]:
                        match += 1

                    if (avgCreeper[0] > cmCreeper[0]):
                        velocidade_saida.publish(velocityCreeperRight)
                    elif (avgCreeper[0] < cmCreeper[0]):
                        velocidade_saida.publish(velocityCreeperLeft)

                    rospy.sleep(0.01)


                # cheguei no creeper, pego ele
            if mode == 'pega creeper':
                velocidade_saida.publish(velocityParked)
                rospy.sleep(0.1)

                # Chama a função para controlar a garra, baseado nos subestados
                controla_garra()
                
                
                

            print("#####################################")
            print('mode: ', mode)
            print('state: ', state)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")