import numpy as np 
import cv2, math

hsv_red = 0 
hsv_yellow = 30
hsv_blue = 120
hsv_pink = 166

inc = 8

hsv = {"red":0 , "yellow" : 30, "blue":120, "pink":166}
dict_masks = {}

def maskValues(color):
    i = "low_"
    j = "high_"
    
    for k,v in hsv.items():
        mask_low = i+k
        mask_high = j+k
    
        if color == "red":
            dict_masks[mask_low]  = np.array([0, 150, 150], dtype=np.uint8)
            dict_masks[mask_high] = np.array([10, 255, 255], dtype=np.uint8)
        else:
            dict_masks[mask_low] = np.array([v-inc, 150, 150], dtype=np.uint8)
            dict_masks[mask_high] = np.array([v+inc, 255, 255], dtype=np.uint8)

    mask_high = j+color
    mask_low  = i+color

    return dict_masks[mask_low], dict_masks[mask_high]