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
import cv_bridge
import auxiliar

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0
direcao = None

frame = "camera_link"

tfl = 0

tf_buffer = tf2_ros.Buffer()

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global state
    global vel
    global direcao

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        imagem1 = temp_image.copy()
        
        media, centro, maior_area = cormodule.identifica_cor(imagem1)
        isCreeper, cmCreeper = auxiliar.searchCreeper(temp_image, cor_creeper, imagem1)
        centro, saida_net, resultados =  visao_module.processa(imagem1)       

        direcao = auxiliar.direction(temp_image, cor_pista, imagem1)

        depois = time.clock()
        cv2.imshow("cv img", imagem1)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

x=None
y=None

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
    global reto
    global chegou
    global dist
    lista_distancias=list(msg.ranges)
    contador_parar=0 

    for i in range(len(lista_distancias)):
        dist = lista_distancias[i]
        if dist <= 0.35:
            chegou = True

    contador_parar = 0
    reto = True 

cv_image = None
media, centro = [], []
reto = False
w=0.3
v=0.3
c = 1
cor_creeper = "blue"  
cor_pista     = "yellow" 
    
if __name__=="__main__":

    rospy.init_node("Projeto")
    topico_img = "/camera/image/compressed"
    odom_sub = rospy.Subscriber("/odom", Odometry, posicao_odometry)
    recebe_img = rospy.Subscriber(topico_img, CompressedImage, roda_todo_frame, queue_size=4, buff_size=2**24)
    scanner    = rospy.Subscriber("/scan", LaserScan, callback)
    vel_saida  = rospy.Publisher("/cmd_vel", Twist, queue_size=3) 
    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    vel = Twist(Vector3(0.2,0,0), Vector3(0,0, 0))
    left = Twist(Vector3(0,0,0), Vector3(0,0, math.pi/15))
    right = Twist(Vector3(0,0,0), Vector3(0,0, -math.pi/15))


    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        rospy.sleep(2.0)
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            vel_saida.publish(vel)

            if direcao == "turn left":
                vel_saida.publish(left)
            elif direcao == "turn right":
                vel_saida.publish(right)
            elif direcao == "straight":
                vel_saida.publish(vel)

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


