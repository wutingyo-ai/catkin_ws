import cv2
import numpy as np 
import time

def nothing(x):

    pass

cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow('image', 900, 600)

blue = np.uint8([[[255, 0, 0]]])

hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)

print(hsv_blue)

cv2.createTrackbar('Hue min', 'image', 0, 255 ,nothing)

cv2.createTrackbar('Hue max', 'image', 0, 255 ,nothing)

cv2.createTrackbar('sat min', 'image', 0, 255 ,nothing)

cv2.createTrackbar('sat max', 'image', 0, 255 ,nothing)

cv2.createTrackbar('val min', 'image', 0, 255 ,nothing)

cv2.createTrackbar('val max', 'image', 0, 255 ,nothing)

img = cv2.imread("/home/chen/save_image/puipui.jpg")

while(1):

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos('Hue min', 'image')

    h_max = cv2.getTrackbarPos('Hue max', 'image')

    s_min = cv2.getTrackbarPos('sat min', 'image')

    s_max = cv2.getTrackbarPos('sat max', 'image')

    v_min = cv2.getTrackbarPos('val min', 'image')

    v_max = cv2.getTrackbarPos('val max', 'image')

    lower = np.array([h_min, s_min, v_min])

    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv_img, lower, upper)

    res = cv2.bitwise_and(img ,img, mask=mask)

    cv2.namedWindow('img',0)
    cv2.resizeWindow('img', 900, 600)

    cv2.namedWindow('mask',0)
    cv2.resizeWindow('mask', 900, 600)

    cv2.namedWindow('res',0)
    cv2.resizeWindow('res', 900, 600)

    cv2.imshow('img',img)

    cv2.imshow('mask',mask)

    cv2.imshow('res',res)

    k = cv2.waitKey(5)

    if k == 27:
        break

cv2.destroyAllWindows()