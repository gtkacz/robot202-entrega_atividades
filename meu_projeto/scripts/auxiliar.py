#!/usr/bin/python
# -*- coding: utf-8 -*-

print("Este script não deve ser executado diretamente")

from ipywidgets import widgets, interact, interactive, FloatSlider, IntSlider
import numpy as np
import cv2
import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet

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

def processa(frame):
    result_frame, result_tuples = mobilenet.detect(frame)
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)

    cross(result_frame, centro, [255,0,0], 1, 17)
    return centro, result_frame, result_tuples

def identifica_cor(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor = np.array([0, 50, 50])
    cor_maior = np.array([8, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    cor_menor = np.array([172, 50, 50])
    cor_maior = np.array([180, 255, 255])
    segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length)



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores
    # que um quadrado 7x7. É muito útil para juntar vários
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    # Encontramos os contornos na máscara e selecionamos o de maior área
    #contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

    return centro, result_frame, result_tuples

def identifica_pista(bgr, color):
    centro = (bgr.shape[1]//2, bgr.shape[0]//2)
    low, high = ranges(color)
    # Valores para amarelo usando um color picker
    # mascara amarela
    low = np.array([22, 50, 50],dtype=np.uint8)
    high = np.array([36, 255, 255],dtype=np.uint8)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    

    # Encontramos os contornos na máscara e selecionamos o de maior área
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    media = None
    if not maior_contorno is None:
        try:
            p = center_of_mass(maior_contorno) # centro de massa
            crosshair(bgr, p, 20, (128,128,0))
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
        
        except:
            p = centro
            crosshair(bgr, p, 20, (128,128,0))

    def booleanContornos(maior_contorno):
        if maior_contorno_area > 100:
            return True
        else:
            return False

    
    return media, centro, booleanContornos(maior_contorno)

def center_of_mass(mask):
    M = cv2.moments(mask)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def center_of_mass_region(mask, x1, y1, x2, y2):
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))

    return mask_bgr

maior_contorno = 0
maior_contorno_area = 0

def identifica_creeper(frame, creeper_color):
    global maior_contorno
    global maior_contorno_area

    segmentado_cor = None
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if creeper_color == "blue":
        cor_menor, cor_maior = aux.ranges("#0000ff")
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    elif creeper_color == "pink":
        cor_menor, cor_maior = aux.ranges("#e60c69")
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 

    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    def booleanContornos(maior_contorno):
        if maior_contorno_area > 100:
            return True
        else:
            return False

    media = None
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 255, 0], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [0, 255,0], 1, 17)
    else:
        media = (0, 0)

    cv2.imshow('seg', segmentado_cor)
    cv2.waitKey(1)

    return media, centro, booleanContornos(maior_contorno)