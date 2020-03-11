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
# !-- SETUP---!
background_path = r'test_images/background.png'
car_path = r'test_images/car2.png'
x = 171
y = 57
# !-- SETUP END---!


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
car_img_resized = cv2.resize(
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)))
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
im_num = 0


background_pil = Image.fromarray(background)
car_img_pil = Image.fromarray(car_img)
background_pil.paste(car_img_pil, (x, y), car_img_pil)
image = np.array(background_pil)

image = cv2.resize(image, (size_w, size_h))
output = image

# pre-process the image for classification
image = cv2.resize(image, (96, 96))
image = image.astype("float") / 255.0
image = img_to_array(image)
image = np.expand_dims(image, axis=0)

# load the trained convolutional neural network and the multi-label
# binarizer
print("[INFO] loading network...")
model = load_model('model')
mlb = pickle.loads(open('labelbin', "rb").read())

# classify the input image then find the indexes of the two class
# labels with the *largest* probability
print("[INFO] classifying image...")
proba = model.predict(image)[0]
idxs = np.argsort(proba)[::-1][:2]
(X, Y, THETA) = ([], [], [])
for i, c in enumerate(mlb.classes_):
    if 'th' in c:
        THETA.append(proba[i])
    elif 'x' in c:
        X.append(proba[i])
    elif 'y' in c:
        Y.append(proba[i])
THETA.sort()
X.sort()
Y.sort()


def inperpolate(CLS, proba, lst_val):
    print('-------')
    CLS = list(CLS)
    proba = list(proba)
    idx1 = proba.index(lst_val[-1])
    idx2 = proba.index(lst_val[-2])
    val1 = float(CLS[idx1].split('_')[-1])
    # val2 = float(CLS[idx2].split('_')[-1])
    # print(val1,val2)
    #p1 = lst_val[-1]/(lst_val[-1]+lst_val[-2])
    # p2 = lst_val[-2]/(lst_val[-1]+lst_val[-2])
    # print(p1,p2)
    # val = val1*p1+val2*p2
    return val1, lst_val[-1]


th, thp = inperpolate(mlb.classes_, proba, THETA)
x, xp = inperpolate(mlb.classes_, proba, X)
y, yp = inperpolate(mlb.classes_, proba, Y)
print('--------[VALUES]--------')
print(round(x, 2), round(y, 2), round(th, 2))
print(round(xp, 2), round(yp, 2), round(thp, 2))
# for i, p in enumerate(proba):
#     if p*100 > 20:
#         print(str(mlb.classes_[i]), str(p*100))
# # loop over the indexes of the high confidence class labels
# for (i, j) in enumerate(idxs):
# 	# build the label and draw the label on the image
# 	label = "{}: {:.2f}%".format(mlb.classes_[j], proba[j] * 100)
# 	cv2.putText(output, label, (10, (i * 30) + 25),
# 		cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

# show the output image
cv2.imshow("Output", output)
cv2.waitKey(0)
