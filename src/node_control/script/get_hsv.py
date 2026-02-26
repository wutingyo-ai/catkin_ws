import cv2

img = cv2.imread("/home/chen/save_image/puipui.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def mouse_click(event, x, y, flags, para):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('PIX:', x, y)
        print("BGR", img[y, x])
        print("GRAY", gray[y, x])
        print("HSV:", hsv[y, x])

if __name__ == '__main__':
    cv2.namedWindow("img", 0)
    cv2.resizeWindow("img", 800, 400)
    cv2.setMouseCallback("img", mouse_click)
    while True:
        cv2.imshow('img', img)
        if cv2.waitKey() == ord('q'):
            break
    cv2.destroyAllWindows()