import os
import random
import csv
import numpy as np
from cv2 import cv2
from scipy import ndimage
from PIL import Image

# !-- SETUP---!
background_path = r'test_images/background.png'
car_path = 'test_images/car2.png'
remove_only = True
# !-- SETUP END---!
DATASET_FOLDER = r"images\\"
TRAIN_OUTPUT_FILE = r"train.csv"

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
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)),interpolation = cv2.INTER_AREA)
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
im_num = 0


filelist = [f for f in os.listdir(DATASET_FOLDER) if f.endswith(".png")]
for f in filelist:
    os.remove(os.path.join(DATASET_FOLDER, f))

if not remove_only:
    with open(TRAIN_OUTPUT_FILE, "w") as train:
        writer = csv.writer(train, delimiter=",")
        for k, thi in enumerate(range(0, 360, 20)):
            car_img = ndimage.rotate(car_img_resized, thi)
            for i, xi in enumerate(range(0, size_w_merge-w_car+1, int(size_w_merge/15))):
                for j, yi in enumerate(range(0, size_h_merge-h_car+1, int(size_h_merge/15))):
                    # rand_int=np.random.randint(8,12)/10
                    # noise_w_car=int(w_car*rand_int)
                    # noise_h_car=int(h_car*rand_int)
                    # car_img = cv2.resize(car_img, (noise_w_car, noise_h_car),interpolation = cv2.INTER_AREA)
                    background_pil = Image.fromarray(background)
                    car_img_pil = Image.fromarray(car_img)
                    background_pil.paste(car_img_pil, (xi,yi), car_img_pil)
                    img=np.array(background_pil) 
                    num = '0'*(5-len(str(im_num)))+str(im_num)
                    xnum = '0'*(4-len(str(xi)))
                    ynum = '0'*(4-len(str(yi)))
                    thnum = '0'*(4-len(str(thi)))
                    path = DATASET_FOLDER+num+".png"
                    row = [path, 'x_'+xnum+'_' +
                           str(xi), 'y_'+ynum+'_'+str(yi), 'th_'+thnum+'_'+str(thi)]
                    writer.writerow(row)
                    img = cv2.resize(img, (size_w, size_h))
                    cv2.imwrite(path, img)
                    im_num += 1
            #         if im_num==100:
            #             break
            #     break
            # break
        print('[Num Images]:',im_num)
