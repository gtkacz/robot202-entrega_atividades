#! /usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function, division

__author__ = ["Gabriel Mitelman Tkacz", "Rafael Selcali Malcervelli", "Enrico Venturini Costa"]

import numpy as np
import numpy
import tf, math, cv2, time, visao_module, rospy, cormodule
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
import pandas as pd

bridge = CvBridge()
cv_image = None
avg = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay = False
rospack = rospkg.RosPack() 

tfl = 0
tf_buffer = tf2_ros.Buffer()
avg=[]
center=[]
isTrack=True
isCreeper=False

x=0
y=0
z=0
theta=0
v=0.2
w=0.2

# Aruco 
ids = None
numero_comparado = 0
match = 0

chegou=False
MobileNetResults=[]
dist=0.3


vel_direita = Twist(Vector3(v,0,0), Vector3(0,0,-w))
vel_esquerda = Twist(Vector3(v,0,0), Vector3(0,0,w))
vel_direita_creeper = Twist(Vector3(v/3,0,0), Vector3(0,0,-w/3))
vel_esquerda_creeper = Twist(Vector3(v/3,0,0), Vector3(0,0,w/3))
vel_frente = Twist(Vector3(v,0,0), Vector3(0,0,0))
vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
vel_girando_ah = Twist(Vector3(0,0,0), Vector3(0,0,w)) # Anti-Horário
vel_girando_h = Twist(Vector3(0,0,0), Vector3(0,0,-w)) # Horário
vel_lenta = Twist(Vector3(0.05,0,0), Vector3(0,0,0))

# Inicializando posicoes da garra e braco
clawIsOpen=-1
clawIsClosed=0
braco_frente = 0
braco_levantado = 1.5
braco_recolhido = -1.5


# Máquina de Estado
estado = None
subestado = None

# Verificação se creeper já foi identificado
identificado = False

# Marcação para guardar a posição anterior à identificação do creeper
flag = True

def posicao_odometry(data):
    global x
    global y
    global theta

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angs_rad = transformations.euler_from_quaternion(lista)
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

    pub.publish(vel_girando_ah)
    rospy.sleep(sleep)

    sleep=0.75/v
    pub.publish(vel_frente)
    rospy.sleep(sleep)

    
def meia_volta():
    global vel_girando_h
    global vel_frente

    tempo1 = 0.75 /v

    # w = dteta/dt
    angulo = math.pi
    tempo2 = angulo/w

    velocidade_saida.publish(vel_girando_h)
    rospy.sleep(tempo2)

    velocidade_saida.publish(vel_frente)
    rospy.sleep(tempo1)


def direcao_robo_pista():
    global subestado

    if subestado == 'frente':
        velocidade_saida.publish(vel_frente)

    if subestado == 'esquerda':
        velocidade_saida.publish(vel_esquerda)

    if subestado == 'direita':
        velocidade_saida.publish(vel_direita)

    if subestado == 'final da pista':
        meia_volta()


def controla_garra():
    global estado
    global subestado
    global identificado

    if subestado == 'levanta garra':
        # Depois de estar a uma distancia pre-estabelecida, o creeper levanta o braco e abre a garra
        # Feito isso, o estado muda para aproxima do creeper
        braco_publisher.publish(braco_frente)
        garra_publisher.publish(clawIsOpen)
        rospy.sleep(0.2)
        subestado = 'aproxima do creeper'

    elif subestado == 'aproxima do creeper':
        velocidade_saida.publish(vel_parado)
        rospy.sleep(0.2)
        tempo_aproxima = (dist - 0.225)/0.05
        velocidade_saida.publish(vel_lenta)
        rospy.sleep(tempo_aproxima)
        # Anda um distancia nao_bateu lentamente 
        #feito isso, muda pra agarra creeper
        subestado = 'agarra creeper'
    
    elif subestado == 'agarra creeper':
        garra_publisher.publish(clawIsClosed)
        rospy.sleep(0.1)
        braco_publisher.publish(braco_levantado)
        rospy.sleep(0.1)
        # Fecha a garra, levanta o braco, muda subestado pra retorna pra pista
        subestado ='retorna pista'        
        
    elif subestado == 'retorna pista':
        go_to(ponto[0], ponto[1], velocidade_saida)
        identificado = True
        estado = 'procurando pista'
        subestado = ""

def roda_todo_frame(img):
    global cv_image
    global avg
    global center
    global MobileNetResults
    global isTrack
    global avgCreeper
    global cmCreeper
    global isCreeper
    global numero_comparado

    now = rospy.get_rostime()
    imgtime = img.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        img1 = temp_image.copy()

        centro, saida_net, MobileNetResults =  visao_module.processa(temp_image)        

        depois = time.clock()
        cv_image = saida_net.copy()
        avg, center, isTrack =  center_mass.identifica_pista(temp_image, "blue")
        avgCreeper, cmCreeper, isCreeper =  creeper.isCreeper(temp_image, goal[0])
        
        # Identificação do Id
        ids = aruco.identifica_id(img1)
        numero_comparado = ids[0][0]

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

    try:
        rospy.sleep(2.0)
        estado = 'procurando pista'
        while not rospy.is_shutdown():
            if estado == 'procurando pista':
                # Se eu nao identifico creeper eu procuro contorno
                if not isCreeper: 
                    while not isTrack:
                        velocidade_saida.publish(vel_girando_ah)
                        rospy.sleep(0.01)
                    # Se eu identificar o contorno mudo meu estado e passo a seguir a pista    
                    estado = 'segue pista'

                # Se eu acho o creeper mudo o estado
                if isCreeper and identificado==False: 
                    estado = 'creeper a la vista'

            if estado == 'segue pista':

                #precisa colocar a centralizacao da pista e o atualizar o subestado de acordo
                #precisa atualizar estado caso veja um creeper
                #precisa atualizar estado caso nao veja mais a pista
                try:
                    if not chegou:
                        print("Encontrei pista")
                        if (avg[0] > center[0]):
                            subestado = 'direita'
                        elif (avg[0] < center[0]):
                            subestado = 'esquerda'
                        else:
                            subestado = 'frente'

                    else: #quando ve o final da pista vira 180 e anda 0.8 pra frente
                        print("Encontrei algum obstáculo")
                        subestado = 'final da pista'
                except:
                    pass

                
                # Mudança de estado
                if not isTrack:
                    estado = 'procurando pista'

                if isCreeper and identificado==False:
                    estado = 'creeper a la vista'  

                # Chama a função direção_robo_pista, que seta a velocidade do robô com base nos substados
                direcao_robo_pista()
                rospy.sleep(0.01)  

            # Creeper foi identificado
            if estado == 'creeper a la vista':

                if flag:
                    ponto = (x,y)
                    print("Ponto gravado:", ponto)
                    flag = False
                    print('Hasta la vista babeeeeee! Ponto: ', ponto) 

                if not chegou: 
                    subestado = 'segue creeper'
                    print('rosa here I goooo')
        
                else:
                    # Se for verdadeiro (match id desejado com id identificado):
                    if match > 200:
                        estado = 'pega creeper'
                        subestado = 'levanta garra' #levanto a garra assim que mudo o estado
                    

                    # Se for falso, volta para a pista:
                    else:
                        print("Entrei na go to")
                        go_to(ponto[0], ponto[1], velocidade_saida)
                        estado = 'procurando pista'
                        match = 0

                
                if subestado == 'segue creeper':
                    print("matchs:", match)
                    
                    if numero_comparado == goal[1]:
                        match += 1

                    if (avgCreeper[0] > cmCreeper[0]):
                        velocidade_saida.publish(vel_direita_creeper)
                    elif (avgCreeper[0] < cmCreeper[0]):
                        velocidade_saida.publish(vel_esquerda_creeper)

                    rospy.sleep(0.01)


                # cheguei no creeper, pego ele
            if estado == 'pega creeper':
                velocidade_saida.publish(vel_parado)
                rospy.sleep(0.1)

                # Chama a função para controlar a garra, baseado nos subestados
                controla_garra()
                
                
                

            print("#####################################")
            print('estado: ', estado)
            print('subestado: ', subestado)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")