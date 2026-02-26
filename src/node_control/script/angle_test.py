import cv2
import numpy as np
import math

bounding_center = []
cen_mass = []

img = cv2.imread("/home/chen/save_image/puipui1.jpg")

kernel1 = np.ones((3,3),np.uint8)

lower_white = np.array([0, 0, 55])
upper_white = np.array([103, 60, 255])

def mask_process(image):

    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_white = cv2.inRange(hsv_img, lower_white, upper_white)
    trans_ori = cv2.bitwise_and(img ,img, mask = mask_white)
    #交集原圖並使用遮罩,變回bgr

    return trans_ori

mask_img = mask_process(img)


gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(3,3), 1)
    
ret ,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
eroded = cv2.erode(th1,kernel1,iterations=1)
dilated = cv2.dilate(eroded,kernel1,iterations = 1)
contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
for c in contours:
    area = cv2.contourArea(c)
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    M =cv2.moments(c)

cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
cv2.circle(img , (cx, cy), 5, (1, 127, 254), -1)

bounding_center = np.array(rect[0])
cen_mass = [cx, cy]

#開口姿態法向量
normal = bounding_center - cen_mass
length = (np.sum(np.square(normal)))**0.5
unit_vec = normal/length
theta = math.atan2(unit_vec[1], unit_vec[0])*180/math.pi

print(bounding_center)
print(cen_mass)
print(normal)
print(length)
print(unit_vec)
print(theta)
print(rect[2])    
print(area)

img_contour = cv2.drawContours(img, [box], -1, (0, 0, 255), 3)


cv2.namedWindow("pic",0)
cv2.resizeWindow("pic", 900, 600)
cv2.imshow("pic",img_contour)
cv2.waitKey(0)