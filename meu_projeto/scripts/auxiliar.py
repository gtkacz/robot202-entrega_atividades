#!/usr/bin/python
# -*- coding: utf-8 -*-

from ipywidgets import widgets, interact, interactive, FloatSlider, IntSlider
import numpy as np
import cv2, masks
from geometry_msgs.msg import Twist, Vector3

def convertToTuple(html_color):
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

def toHsv(html_color):
    tupla = convertToTuple(html_color)
    hsv = cv2.cvtColor(to_1px(tupla), cv2.COLOR_RGB2HSV)
    return hsv[0][0]

def ranges(value):
    hsv = toHsv(value)
    hsv2 = np.copy(hsv)
    hsv[0] = max(0, hsv[0]- 30)
    hsv2[0] = min(180, hsv[0]+ 30)
    hsv[1:] = 50
    hsv2[1:] = 255
    return hsv, hsv2

def massCenter(mask):
    M = cv2.moments(mask)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def colorFilter(bgr, low, high):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask  

def massCenterColor(img_bgr_limpa, str_cor, img_bgr_visivel):
    hsv_low, hsv_high = masks.maskValues(str_cor)
    color_mask = colorFilter(img_bgr_limpa, hsv_low, hsv_high)

    if str_cor == "amarelo":
        color_mask = restrictWindowSize(img_bgr_limpa, color_mask, img_bgr_visivel, [30,60])

    posicao_centro_massa = massCenter(color_mask) 
    desenha_centro = crosshair(img_bgr_visivel, posicao_centro_massa, 8, (255,0,255))
    return posicao_centro_massa

def restrictWindowSize(img_bgr_limpa, mascara, img_bgr_visivel, list_xo_y0):
    x0 = list_xo_y0[0]
    y0 = 90
    x1 = img_bgr_limpa.shape[1] - 10
    y1 = img_bgr_limpa.shape[0] - 20

    clipped = mascara[y0:y1, x0:x1]
    cv2.rectangle(img_bgr_visivel, (x0, y0), (x1, y1), (255,0,0),2,cv2.LINE_AA) 
    return clipped

def direction(img_bgr_limpa, str_cor, img_bgr_visivel):
    try:
        cm = massCenterColor(img_bgr_limpa, str_cor, img_bgr_visivel)
        x_centro  = cm[0]

        if x_centro < centro - incerteza:
            return "turn left"
        elif x_centro > centro + incerteza:
            return "turn right"
        else:
            return "straight"

    except:
        return "perdeu pista"

def moveToCenterOfMass(qual_direcao, velocidade_atual, vel_lin, vel_ang):
    if   qual_direcao == "seguir reto":
        velocidade_atual.angular.z = 0
        if velocidade_atual.linear.x <= 0.2:
            velocidade_atual.linear.x = 0.25
        if velocidade_atual.linear.x <= 2*vel_lin:
            velocidade_atual.linear.x += 0.1*vel_lin
        return velocidade_atual

    elif qual_direcao == "virar direita":
        velocidade_atual.angular.z = -vel_ang
        return velocidade_atual

    elif qual_direcao == "virar esquerda":
        velocidade_atual.angular.z = vel_ang
        return velocidade_atual

    elif qual_direcao == "perdeu pista":
        velocidade_atual.linear.x = 0
        velocidade_atual.angular.z = -3*vel_ang
        return velocidade_atual

    elif qual_direcao == 'rotate_until_is_creeper_visible':
        velocidade_atual.linear.x = 0
        velocidade_atual.angular.z = -3*vel_ang
        
        return velocidade_atual

def restrictWindow(img_bgr_limpa, list_xo_y0):
    x0 = list_xo_y0[0]
    y0 = list_xo_y0[1]
    x1 = img_bgr_limpa.shape[1] - 10
    y1 = img_bgr_limpa.shape[0] - y0

    clipped = img_bgr_limpa[y0:y1, x0:x1]
    return clipped

def searchCreeper(img_bgr_limpa, cor_creeper, img_bgr_visivel): 
    hsv_low, hsv_high  = masks.maskValues(cor_creeper)
    is_creeper_visible = False
    color_mask_creeper = colorFilter(img_bgr_limpa, hsv_low, hsv_high)

    try: 
        posicao_centro_massa_creeper = massCenter(color_mask_creeper) 
        desenha_centro = crosshair(img_bgr_visivel, posicao_centro_massa_creeper, 8, (255,100,100))
        is_creeper_visible = True
        return is_creeper_visible, posicao_centro_massa_creeper
    except:
        return is_creeper_visible, (0,0)      

incerteza = 15
centro    = 160
def moveToCreeper(posicao_centro_massa_creeper):
    try:
        x_centro  = posicao_centro_massa_creeper[0]

        if x_centro < centro - incerteza:
            return "virar esquerda"
        elif x_centro > centro + incerteza:
            return "virar direita"
        else:
            return "seguir reto"

    except:
        return "perdeu pista"

