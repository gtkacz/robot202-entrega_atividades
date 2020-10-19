#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__      = "Matheus Dib, Fabio de Miranda"


import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import sys
import auxiliar as aux
import math
#import imutils
#from imutils import paths

# If you want to open a video, just change v2.VideoCapture(0) from 0 to the filename, just like below


if len(sys.argv) > 1:
    arg = sys.argv[1]
    try:
        input_source=int(arg) # se for um device
    except:
        input_source=str(arg) # se for nome de arquivo
else:   
    input_source = 0

cap = cv2.VideoCapture(input_source)


# Parameters to use when opening the webcam.


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

print("Press q to QUIT")

# Returns an image containing the borders of the image
# sigma is how far from the median we are setting the thresholds
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

D = 40
H = float(input("Digite a distância real entre os centros dos círculos em cm: "))
h = 560 
f = (h*D)/H



while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the frame to grayscale and hsv
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    magenta='#FF00FE'
    hsv1_m, hsv2_m = aux.ranges(magenta)
    mask_m = cv2.inRange(hsv, hsv1_m, hsv2_m)

    ciano='#00FFFF'
    hsv1_c, hsv2_c = aux.ranges(ciano)
    mask_c = cv2.inRange(hsv, hsv1_c, hsv2_c)

    
    blur_mag = cv2.GaussianBlur(mask_m,(5,5),0)
    blur_cya = cv2.GaussianBlur(mask_c,(5,5),0)
    blur = cv2.add(blur_cya, blur_mag) 

    # Detect the edges present in the image
    bordas_m = auto_canny(blur_mag)
    bordas_c = auto_canny(blur_cya)

    bordas = cv2.add(bordas_c, bordas_m)
    
    
    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)   

    circles_mag = []
    circles_cya = []
    
    circles_mag = None
    xm = None
    ym = None
    circles_m = cv2.HoughCircles(bordas_m,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=5,maxRadius=60)

    if circles_m is not None:        
        circles_m = np.uint16(np.around(circles_m))
        for i in circles_m[0,:]:
            xm = i[0]
            ym = i[1]

            # draw the outer circle
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(255,0,255),2)
            # draw the center of the circle
            cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)
          
            # text showing what color is the circle.
            cv2.putText(bordas_color,'Magenta',(i[0], i[1]), font, 1,(255,0,255),2,cv2.LINE_AA)

            
    center_mag = tuple([xm, ym])
   
    circles_cya = None
    xc = None
    yc = None
    circles_c = cv2.HoughCircles(bordas_c,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=5,maxRadius=60)

    if circles_c is not None:        
        circles_c = np.uint16(np.around(circles_c))
        for i in circles_c[0,:]:
            xc = i[0]
            yc = i[1]

            # draw the outer circle
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(255,255,0),2)
            # draw the center of the circle
            cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)

            # text showing what color is the circle.
            cv2.putText(bordas_color,'Ciano',(i[0], i[1]), font, 1,(255,255,0),2,cv2.LINE_AA)

            
    center_cyan = tuple([xc, yc])
    
    
    ang = None

    if center_mag[0] != None and center_mag[1] != None or center_cyan[0] != None and center_cyan[1] != None:
        try:
            cv2.line(bordas_color, center_mag, center_cyan, (255, 255, 255), 6)
            h=abs(center_mag[1] - center_cyan[1] + center_mag[0] - center_cyan[0])
            d = (H*f)/h
            cv2.putText(bordas_color,'Distance -> %.1f' %d,(0,300), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
            ang = math.atan((ym - yc) / (xm - xc))
        except:
            pass

    if ang != None:
        cv2.putText(bordas_color,'Angle -> %.1f' % (math.degrees(ang)),(0,450), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)

    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(bordas_color,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    
    # Display the resulting frame
    cv2.imshow('Detector de circulos',bordas_color)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#  When everything done, release the capture
cap.release()
cv2.destroyAllWindows()