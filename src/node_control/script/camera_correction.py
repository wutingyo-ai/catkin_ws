import cv2
import numpy as np
import math

cen_mass = []
kernel1 = np.ones((3,3),np.uint8)
img = cv2.imread("/home/chen/save_image/pixel_to_mm/ori_img.jpg")
imgx = cv2.imread("/home/chen/save_image/pixel_to_mm/x_img.jpg")
imgy = cv2.imread("/home/chen/save_image/pixel_to_mm/y_img.jpg")

def find_mass(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3), 1)
    
    ret ,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    eroded = cv2.erode(th1,kernel1,iterations=1)
    dilated = cv2.dilate(eroded,kernel1,iterations = 1)
    contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

    
    for c in contours:
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        M =cv2.moments(c)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cen_mass = [cx, cy]

        return cen_mass
    
ori_mas = find_mass(img)
print(ori_mas)
x_mas = find_mass(imgx)
print(x_mas)
y_mas = find_mass(imgy)
print(y_mas)