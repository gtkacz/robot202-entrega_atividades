#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import rospkg
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float64
import auxiliar
import aruco

bridge = CvBridge() 
cv_image = None 
rospack = rospkg.RosPack() 
#Objetivo
#         cor   id  estacao
goal = ("rosa", 13, "bird")

media_pista = []
centro_pista = []
identifica_contorno_pista = True
identifica_creeper = False

# Atrasos e delays - Descarta imagens que chegam atrasadas demais
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados.  
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay = False
resultados = []
x = 0
y = 0
z = 0 
alpha = 0
ids = None
numero_comparado = 0
match = 0
tfl = 0
tf_buffer = tf2_ros.Buffer()
nao_bateu = True
dist_preventiva = 0.3

# Inicializando velocidades - por default gira no sentido anti-horário
w = 0.35
v = 0.25

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
braco_frente = 0
braco_levantado = 1.5
braco_recolhido = -1.5

garra_aberta = -1
garra_fechada = 0


# Máquina de Estado
estado = None
subestado = None

# Verificação se creeper já foi identificado
identificado = False

# Marcação para guardar a posição anterior à identificação do creeper
flag = True


# FUNÇÕS A SEREM UTILIZADAS

def scaneou(dado):
    global nao_bateu
    global dist_preventiva
    
    if dado.ranges[0] <= dist_preventiva:
        nao_bateu = False
    else:
        nao_bateu = True


def recebe_odometria(data):
    global x
    global y
    global alpha

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)

    alpha = angulos_rad[2] # mais facil se guardarmos alpha em radianos
    

def go_to(x2, y2, pub):
    # Odometria
    global alpha 
    global x
    global y

    # Velocidades 
    global v
    global w
    global vel_girando_ah
    global vel_frente
    
    # calcular theta
    theta = math.atan2(y2-y, x2-x)

    # ângulo que o robô deve virar para se ajustar com o ponto
    angulo = theta - alpha

    # girar theta - alpha para a esquerda
    tempo = angulo / w


    # corrigir o ângulo
    pub.publish(vel_girando_ah)
    rospy.sleep(tempo)

    # andar 0.8 metros
    tempo2 = 0.8 / v
    pub.publish(vel_frente)
    rospy.sleep(tempo2)

    
def meia_volta():
    global vel_girando_h
    global vel_frente

    tempo1 = 0.8 /v

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
        garra_publisher.publish(garra_aberta)
        rospy.sleep(0.2)
        subestado = 'aproxima do creeper'

    elif subestado == 'aproxima do creeper':
        velocidade_saida.publish(vel_parado)
        rospy.sleep(0.2)
        tempo_aproxima = (dist_preventiva - 0.225)/0.05
        velocidade_saida.publish(vel_lenta)
        rospy.sleep(tempo_aproxima)
        # Anda um distancia nao_bateu lentamente 
        #feito isso, muda pra agarra creeper
        subestado = 'agarra creeper'
    
    elif subestado == 'agarra creeper':
        garra_publisher.publish(garra_fechada)
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
    


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media_pista
    global centro_pista
    global resultados
    global identifica_contorno_pista

    global media_creeper
    global centro_creeper
    global identifica_creeper

    global numero_comparado

    # Para robô real
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        aruco_image = temp_image.copy()

        # Note que os resultados já são guardados automaticamente na variável chamada resultados
        centro, saida_net, resultados =  auxiliar.processa(temp_image)        
        for r in resultados:
            # print(r) - print dos resultados da mobilebnet
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas

        cv_image = saida_net.copy()
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
        media_pista, centro_pista, identifica_contorno_pista =  auxiliar.identifica_pista(temp_image)
        media_creeper, centro_creeper, identifica_creeper =  auxiliar.identifica_creeper(temp_image, goal[0])
        
        # Identificação do Id
        ids = aruco.identifica_id(aruco_image)
        numero_comparado = ids[0][0]

        cv2.imshow("cv_image", temp_image)
        cv2.imshow("aruco_image", aruco_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)
    path = rospack.get_path('meu_projeto')


if __name__=="__main__":
    rospy.init_node("main") 

    topico_imagem = "/camera/image/compressed"
    # topico_imagem = "/raspicam/image_raw/compressed" # Use para robo real

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    braco_publisher = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=1)
    garra_publisher = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=1)


    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    
    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]


    try:
        estado = 'procurando pista'

        while not rospy.is_shutdown():
            # for r in resultados:
            #    print(r)

            if estado == 'procurando pista':
                # Se eu nao identifico creeper eu procuro contorno
                if not identifica_creeper: 
                    while not identifica_contorno_pista:
                        velocidade_saida.publish(vel_girando_ah)
                        rospy.sleep(0.01)
                    # Se eu identificar o contorno mudo meu estado e passo a seguir a pista    
                    estado = 'segue pista'

                # Se eu acho o creeper mudo o estado
                if identifica_creeper and identificado==False: 
                    estado = 'creeper a la vista'

            if estado == 'segue pista':

                #precisa colocar a centralizacao da pista e o atualizar o subestado de acordo
                #precisa atualizar estado caso veja um creeper
                #precisa atualizar estado caso nao veja mais a pista
                try:
                    if nao_bateu:
                        print("Encontrei pista")
                        if (media_pista[0] > centro_pista[0]):
                            subestado = 'direita'
                        elif (media_pista[0] < centro_pista[0]):
                            subestado = 'esquerda'
                        else:
                            subestado = 'frente'

                    else: #quando ve o final da pista vira 180 e anda 0.8 pra frente
                        print("Encontrei algum obstáculo")
                        subestado = 'final da pista'
                except:
                    pass

                
                # Mudança de estado
                if not identifica_contorno_pista:
                    estado = 'procurando pista'

                if identifica_creeper and identificado==False:
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

                if nao_bateu: 
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

                    if (media_creeper[0] > centro_creeper[0]):
                        velocidade_saida.publish(vel_direita_creeper)
                    elif (media_creeper[0] < centro_creeper[0]):
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