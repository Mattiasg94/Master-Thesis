import numpy as np
from cv2 import cv2
from openCV import images

X=[]
Y=[]
orig_img = cv2.imread('car.png')
for i in range(2):
    img = cv2.imread(str(images[i])+'.png')
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 255, 255, 255)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    radius_lst=[]
    X_lst=[]
    Y_lst=[]
    for ctr in contours:
        (x,y),radius = cv2.minEnclosingCircle(ctr)
        x,y = (int(x),int(y))
        radius_lst.append(int(radius))
        X_lst.append(x)
        Y_lst.append(y)

    idx=radius_lst.index(max(radius_lst))
    radius=(radius_lst[idx])
    X.append(X_lst[idx])
    Y.append(Y_lst[idx])

    cv2.circle(orig_img,(X[i],Y[i]),radius,(0,255,0),2)
    cv2.circle(orig_img,(X[i],Y[i]),4,(0,255,0),3)

    # cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    # cv2.drawContours(imgray, contours, -1, (0, 255, 0), 3)


d=np.sqrt(np.power(X[0]-X[1],2)+np.power(Y[0]-Y[1],2))
angle=np.arctan2(Y[0]-Y[1],X[0]-X[1])

xs = X[0] - np.cos(angle)*d/2
ys = Y[0] - np.sin(angle)*d/2
cv2.circle(orig_img,(int(xs),int(ys)),4,(0,255,0),3)

print(d)
print(angle)
cv2.imshow('Image', orig_img)
cv2.waitKey(0)
cv2.destroyAllWindows()