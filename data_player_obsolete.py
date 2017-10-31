# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 10:19:46 2017

@author: Mfahad
"""



# related third party imports
import numpy as np
import cv2
import pandas as pd
import math
np.set_printoptions(precision=14)

# Imports from within the repo
from pedestrian import Pedestrian

ind = 0
t_splice = 0
img = cv2.imread('scene_map.png',-1)

headers = ["time_stamp","ped_id","x","y","z","vel","ang_m","ang_f"]

def subimage(image, center, theta, width, height):
   theta *= 3.14159 / 180 # convert to rad

   v_x = (math.cos(theta), math.sin(theta))
   v_y = (-1*math.sin(theta), math.cos(theta))
   s_x = center[0] - v_x[0] * (width / 2) - v_y[0] * (height / 2)
   s_y = center[1] - v_x[1] * (width / 2) - v_y[1] * (height / 2)

   mapping = np.array([[v_x[0],v_y[0], s_x],[v_x[1],v_y[1], s_y]])

   return cv2.warpAffine(image,mapping,(width, height),flags=cv2.WARP_INVERSE_MAP,borderMode=cv2.BORDER_REPLICATE)

def contains(obj_list, ped_id):
    cnt = 0
    for x in obj_list:
        if x.ID == ped_id:
            return cnt
        cnt = cnt+1
    return -1
c1 = 0
def render_scene():
    global img,ped_list,c1
    img2=img.copy()
    for c in range(0,(len(ped_list))):
        #cv2.polylines(img2, np.int32([ped_list[c].points_px]), 5, (100,100,100))
        #cv2.circle(img2,(ped_list[c].x,ped_list[c].y), 5, (0,255,0), -1)
        cv2.ellipse(img2,(ped_list[c].x,ped_list[c].y),(ped_list[c].pix_w,ped_list[c].pix_h),360-ped_list[c].degrees(),0,360,(0,255,0),-1)#(img2,(ped_list[c].x,ped_list[c].y),(ped_list[c].pix_w,ped_list[c].pix_h), ped_list[c].ang_mo,0,360,255, -1)
        print (ped_list[c].x,ped_list[c].y,ped_list[c].ang_mo[-1])
        
    for c in range(0,(len(ped_list))):
        cv2.ellipse(img2,(ped_list[c].x,ped_list[c].y),(ped_list[c].pix_w,ped_list[c].pix_h),360-ped_list[c].degrees(),0,360,(255,0,0),-1)
        sub_img = subimage(img2, (ped_list[c].x,ped_list[c].y), 360-ped_list[c].degrees(), 256, 256)
        cv2.ellipse(img2,(ped_list[c].x,ped_list[c].y),(ped_list[c].pix_w,ped_list[c].pix_h),360-ped_list[c].degrees(),0,360,(0,255,0),-1)        
        c1=c1+1
        #cv2.imwrite('data/'+str(c1)+'.png',sub_img)
    cv2.imshow('image',img2)
    cv2.imwrite('fahad_traj.png',img2)
    cv2.waitKey(1)
        
def read_next_frame():
    global ind

    data_no_headers = pd.read_csv("atc-20121114.csv", names = headers, skiprows=ind, nrows=1)        
    d_frame = data_no_headers.values #generate data frame from csv

    while d_frame[0,0]==data_no_headers.values[0,0]:
        ind=ind+1
        data_no_headers = pd.read_csv("atc-20121114.csv", names = headers, skiprows=ind, nrows=1)
        if d_frame[0,0]==data_no_headers.values[0,0]:
            d_frame = np.concatenate((d_frame,data_no_headers.values), axis=0)

    return d_frame
ped_list = []
a = read_next_frame()
print(a.shape)
a_row = a.shape[0]
#a_col = a.shape[1]
print(a_row)
for count in range(0,a.shape[0]):
   t1 = Pedestrian(a[count,1],a[count,2],a[count,3],a[count,4],a[count,5],a[count,6],a[count,7],a[count,0])
   ped_list.append(t1)
   print(count)

for count1 in range(0,20000):
    a = read_next_frame()
    for count in range(0,a.shape[0]):
        indx = contains(ped_list,a[count,1])
        if indx>=0:
            ped_list[indx].add_point(a[count,2],a[count,3],a[count,4],a[count,5],a[count,6],a[count,7],a[count,0])
        if indx==-1:
            t1= Pedestrian(a[count,1],a[count,2],a[count,3],a[count,4],a[count,5],a[count,6],a[count,7],a[count,0])
            t1.updated = 1
            ped_list.append(t1)
    print (len(ped_list))
    b = 0
    for x in ped_list:
        if x.updated == 1:
            ped_list[b].updated = 0
        else:
            ped_list.remove(x)
        b=b+1
     # do something with it
    render_scene()
            
#a = read_next_frame()
#print t1

#cv2.imshow('image',img)
#k = cv2.waitKey(0)
#if k == 27:         # wait for ESC key to exit
    #cv2.destroyAllWindows()
