#!/usr/bin/python
# -*- coding: utf-8 -*-

print("Este script não deve ser executado diretamente")

from ipywidgets import widgets, interact, interactive, FloatSlider, IntSlider
import numpy as np
import cv2, masks, rospy, time
import cv2.aruco as aruco 
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64

def make_widgets_mat(m, n):
    """
        Makes a m rows x n columns 
        matriz of  integer Jupyter Widgets
        all values initialized to zero
    """
    list_elements = []
    for i in range(m):
        row = []
        for j in range(n):
            row.append(widgets.IntText(value=0))
        list_elements.append(row)

    rows = []
    for row in list_elements:
        rows.append(widgets.HBox(row))

    widgets_mat = widgets.VBox(rows)

    return list_elements, widgets_mat

def make_widgets_mat_from_data(data):
    """
        Creates a matriz of int Widgets given 2D-data
    """
    n = len(data)
    m = len(data[0])
    elements, mat = makeMat(n, m)
    for i in range(n):
        for j in range(m):
            elements[i][j].value = data[i][j]
    return elements, mat

def make_np_from_widgets_list(widgets_list):
    """
        Takes as input a list of lists of widgets and initializes a matrix
    """
    widgets = widgets_list
    n = len(widgets)
    m = len(widgets[0])
    array = np.zeros((n,m), dtype=np.float32)
    for i in range(n):
        for j in range(m):
            array[i][j] = widgets[i][j].value
    return array


def convert_to_tuple(html_color):
    colors = html_color.split("#")[1]
    r = int(colors[0:2],16)
    g = int(colors[2:4],16)
    b = int(colors[4:],16)
    return (r,g,b)

def to_1px(tpl):
    img = np.zeros((1,1,3), dtype=np.uint8)
    img[0,0,0] = tpl[0]
    img[0,0,1] = tpl[1]
    img[0,0,2] = tpl[2]
    return img

def to_hsv(html_color):
    tupla = convert_to_tuple(html_color)
    hsv = cv2.cvtColor(to_1px(tupla), cv2.COLOR_RGB2HSV)
    return hsv[0][0]

def ranges(value):
    hsv = to_hsv(value)
    hsv2 = np.copy(hsv)
    hsv[0] = max(0, hsv[0]- 30)
    hsv2[0] = min(180, hsv[0]+ 30)
    hsv[1:] = 50
    hsv2[1:] = 255
    return hsv, hsv2

def colorNameToValue(color):
    if color == 'blue':
        return '#'
    elif color == 'orange':
        return '#'
    elif color == 'pink':
        return '#'

def seleciona_cor(bgr, low, high):
    cor_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(cor_hsv, low, high)
    kernel = np.ones((6,6),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask  

def cm(mask):
    M = cv2.moments(mask)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def encontra_hsvs(cor):
    if cor == "yellow":
        yellow_hsv = 30
        low  = np.array([(yellow_hsv - 10), 150, 150], dtype=np.uint8)
        high = np.array([(yellow_hsv + 10), 255, 255], dtype=np.uint8)

    elif cor == "orange":
        low  = np.array([0, 150, 150], dtype=np.uint8)
        high = np.array([10, 255, 255], dtype=np.uint8)

    elif cor == "blue":
        blue_hsv = 120
        low  = np.array([(blue_hsv - 10), 150, 150], dtype=np.uint8)
        high = np.array([(blue_hsv + 10), 255, 255], dtype=np.uint8)
    
    elif cor == "pink":
        pink_hsv = 165
        low  = np.array([(pink_hsv - 10), 150, 150], dtype=np.uint8)
        high = np.array([(pink_hsv + 10), 255, 255], dtype=np.uint8)

    return low, high

 
def encontra_cm(bgr, cor_escolhida):
    low, high = encontra_hsvs(cor_escolhida)
    mascara   = seleciona_cor(bgr, low, high)
    centro_massa = cm(mascara)
    draw_cm = crosshair(bgr, centro_massa,  15, (200,100,255))
    return centro_massa


def controla_direcao(bgr, cor_escolhida):
    try:
        cm = encontra_cm(bgr, cor_escolhida)
        x_cm  = cm[0]
        incerteza = 20
        centro    = 160

        if x_cm < centro - incerteza:
            return "go left"
        elif x_cm > centro + incerteza:
            return "go right"
        else:
            return "reto"
    except:
        return "perdeu pista"

def muda_velocidade(qual_direcao, velocidade_atual, vel_lin, vel_ang):
    if   qual_direcao == "reto":
        velocidade_atual.angular.z = 0
        velin = velocidade_atual.linear.x
        if velin <= 0.4:
            velocidade_atual.linear.x += 0.03
        return velocidade_atual

    elif qual_direcao == "go right":
        velocidade_atual.angular.z = -vel_ang
        return velocidade_atual

    elif qual_direcao == "go left":
        velocidade_atual.angular.z = vel_ang
        return velocidade_atual

    elif qual_direcao == "perdeu pista":
        velocidade_atual.linear.x = 0
        velocidade_atual.angular.z = -2.2*vel_ang
        return velocidade_atual


######################## IDENTIFICA COM ARUCO ###########################

#--- Get the camera calibration path
calib_path          = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

marker_size  = 25 #centimetros
ids_fim_pista = [50,100,150]
menor_distancia_tag = 1300

def identifica_tag(img_bgr):
    try: 
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            aruco.drawDetectedMarkers(img_bgr, corners, ids) 
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            distancia_tag = np.linalg.norm(tvec)

            if distancia_tag < menor_distancia_tag:
                menor_distancia = distancia_tag   
        
        return menor_distancia, corners, ids
    except:
        return None, None, None

def rotacionar_procurar_creeper(img_bgr):
    menor_distancia, corners, ids = identifica_tag(img_bgr)
    if ids is not None:
        for tag in ids:
            print(tag)
            print(menor_distancia)
            if tag[0] in ids_fim_pista and menor_distancia <= 250:
                print("ROTACIONAR PARA ENCONTRAR O CREEPER")
                return True
    else: 
        return False

def distacia_ate_creeper(img_bgr, id_creeper_procurado):  
    menor_distancia, corners, ids = identifica_tag(img_bgr)
    if ids is not None:
        for tag in ids:
            print("distancia creeper: ")
            print(menor_distancia, "\n")
            if tag[0] == id_creeper_procurado and menor_distancia <= 213:
                return True
    else: 
        return False


######################## GARRA ###########################

class garra:
    def __init__(self):
        self.arm_publisher = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=1)
        self.garra_publisher = rospy.Publisher('/joint2_position_controller/command',Float64,queue_size=1)

        self.arm_state = -1
        self.garra_state = 0
        self.time = 0.1

    def inicializar_garra(self):
        self.arm_state = -1
        self.garra_state =  0
        
        self.arm_publisher.publish(self.arm_state) #abaixa 
        rospy.sleep(self.time)

        self.garra_publisher.publish(self.garra_state) #fecha 
        rospy.sleep(self.time)

    def open(self):
        self.garra_state = -1
        self.garra_publisher.publish(self.garra_state)
        rospy.sleep(self.time)

    def close(self):
        self.garra_state = 0
        self.garra_publisher.publish(self.garra_state)
        rospy.sleep(self.time)
        
    def up(self):
        if self.arm_state == -1:
            self.arm_state = 0
        elif self.arm_state == 0:
            self.arm_state = 1.5
        
        self.arm_publisher.publish(self.arm_state)
        rospy.sleep(self.time)

    def down(self):
        if self.arm_state == 1.5:
            self.arm_state = 0
        elif self.arm_state == 0:
            self.arm_state = -1
        
        self.arm_publisher.publish(self.arm_state)
        rospy.sleep(self.time)
