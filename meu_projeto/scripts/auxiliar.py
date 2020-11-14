#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    Atenção: usado no notebook da aula. 
    Não precisa ser usado diretamente
"""

print("Este script não deve ser executado diretamente")

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

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
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
    hsv_low, hsv_high = masks.criar_valores_mascaras(str_cor)
    color_mask = filter_color(img_bgr_limpa, hsv_low, hsv_high)

    if str_cor == "amarelo":
        color_mask = restringir_window_size(img_bgr_limpa, color_mask, img_bgr_visivel, [30,60])

    posicao_centro_massa = center_of_mass(color_mask) 
    desenha_centro = crosshair(img_bgr_visivel, posicao_centro_massa, 8, (255,0,255))
    return posicao_centro_massa

def restrictWindowSize(img_bgr_limpa, mascara, img_bgr_visivel, list_xo_y0):
    x0 = list_xo_y0[0]
    # y0 = list_xo_y0[1]
    y0 = 90
    x1 = img_bgr_limpa.shape[1] - 10
    y1 = img_bgr_limpa.shape[0] - 20

    clipped = mascara[y0:y1, x0:x1]
    cv2.rectangle(img_bgr_visivel, (x0, y0), (x1, y1), (255,0,0),2,cv2.LINE_AA) #desenha retangulo da área selecionada
    return clipped

def direction(img_bgr_limpa, str_cor, img_bgr_visivel):
    try:
        cm = centro_massa_cor(img_bgr_limpa, str_cor, img_bgr_visivel)
        x_centro  = cm[0]

        if x_centro < centro - incerteza:
            return "virar esquerda"
        elif x_centro > centro + incerteza:
            return "virar direita"
        else:
            return "seguir reto"

    except:
        return "perdeu pista"

