import cv2
import numpy as np

scale = 2
delta = 0
ddepth = cv2.CV_16S
cap = cv2.VideoCapture(0)

while(1):
    _, img = cap.read()
    img = cv2.GaussianBlur(img,(5,5),0)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    kernel = np.ones((3,3),np.uint8)
    grad_x = cv2.Sobel(gray,ddepth,1,0,ksize = 3, scale = scale, delta = delta,borderType = cv2.BORDER_DEFAULT)
    #grad_x = cv2.Scharr(gray,ddepth,1,0)
    grad_y = cv2.Sobel(gray,ddepth,0,1,ksize = 3, scale = scale, delta = delta, borderType = cv2.BORDER_DEFAULT)
    #grad_y = cv2.Scharr(gray,ddepth,0,1)

    abs_grad_x = cv2.convertScaleAbs(grad_x) 
    abs_grad_y = cv2.convertScaleAbs(grad_y)

    dst = cv2.addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0)
   #dst = cv2.add(abs_grad_x,abs_grad_y)
    dst1 = cv2.erode(dst,kernel,iterations = 2)
    dst2 = cv2.dilate(dst,kernel,iterations = 5)
    cv2.imshow('SOBEL',dst)
    cv2.imshow('erode',dst1)
    cv2.imshow('dilate',dst2)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
