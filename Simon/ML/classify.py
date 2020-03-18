# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import argparse
import imutils
import pickle
from cv2 import cv2
import os
from PIL import Image
from scipy import ndimage
# !-- SETUP---!
background_path = r'test_images/background.png'
car_path = r'test_images/car2.png'
x = 195
y = 75
th=10
# !-- SETUP END---!
# model_pos = load_model('model_pos')
# mlb_pos = pickle.loads(open('labelbin_pos', "rb").read())

model_theta = load_model('model_theta')
mlb_theta = pickle.loads(open('labelbin_theta', "rb").read())

size_w = 96
size_h = 96
size_w_merge = size_w*3
size_h_merge = size_h*3
background_orig = cv2.imread(background_path)
w_back_orig = background_orig.shape[1]
h_back_orig = background_orig.shape[0]
background = cv2.resize(background_orig, (size_w_merge, size_w_merge))
resize_ratio_w = size_w_merge/w_back_orig
resize_ratio_h = size_w_merge/h_back_orig

car_img = cv2.imread(car_path, cv2.IMREAD_UNCHANGED)
w_car_orig = car_img.shape[1]
h_car_orig = car_img.shape[0]
car_img = ndimage.rotate(car_img, th)
car_img_resized = cv2.resize(
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)))
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
background_pil = Image.fromarray(background)
car_img_pil = Image.fromarray(car_img)
background_pil.paste(car_img_pil, (x, y), car_img_pil)
image = np.array(background_pil)
hejpa=image
image = cv2.resize(image, (size_w, size_h))
output = image

def inperpolate(CLS, proba, lst_val):
    CLS = list(CLS)
    proba = list(proba)
    idx1 = proba.index(lst_val[-1])
    idx2 = proba.index(lst_val[-2])
    val1 = float(CLS[idx1].split('_')[-1])
    val2 = float(CLS[idx2].split('_')[-1])
    if lst_val[-2]>0.7:  
        p1 = lst_val[-1]/(lst_val[-1]+lst_val[-2])
        p2 = lst_val[-2]/(lst_val[-1]+lst_val[-2])
        val = val1*p1+val2*p2
        return val, lst_val[-1]
    else:
        return val1, lst_val[-1]

image = cv2.resize(image, (int(w_car*1.2), int(h_car*1.2)))
cv2.imwrite('image.jpg',image)
image=cv2.imread(r'theta_images/00000.png')
image = image.astype("float") / 255.0
image = img_to_array(image)
image = np.expand_dims(image, axis=0)
THETA=[]
proba_theta = model_theta.predict(image)[0]
for i, c in enumerate(mlb_theta.classes_):
    THETA.append(proba_theta[i])
    if (proba_theta[i]*100)>10:
        pass
    print(mlb_theta.classes_[i],proba_theta[i]*100)
THETA.sort()
th, thp = inperpolate(mlb_theta.classes_, proba_theta, THETA)

print('--------[VALUES]--------')
# print(round(x, 2), round(y, 2), round(th, 2))
# print(round(xp, 2), round(yp, 2), round(thp, 2))
print(round(th, 2))
print(round(thp, 2))

