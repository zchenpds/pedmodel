#file -- Pedestrian.py --
"""
Author: Mfahad
"""
import numpy as np
import math

class Pedestrian():
    def __init__(self,ID,x_pos,y_pos,z_height,vel,ang_mo,ang_fc,ts):
        self.ID = ID
        self.points = np.array([[x_pos,y_pos]]) # position
        self.points_px = np.array([[int((x_pos - (-1*60000))/50) , int((20000 - (y_pos))/50)]])
        self.x = int((x_pos - (-1*60000))/50)
        self.y = int((20000 - (y_pos))/50)
        self.z_height = np.array([z_height])
        ped_wid = int(round(self.z_height/(50*pow(1.61,3))))
        self.pix_w = ((ped_wid%2==0)*(ped_wid/2))+((ped_wid%2==1)*((ped_wid+1)/2))
        self.pix_h = ((self.pix_w%2==0)*(self.pix_w/2))+((self.pix_w%2==1)*((self.pix_w+1)/2))
        self.vel = np.array([vel])
        self.ang_mo = np.array([ang_mo])
        self.ang_fc = np.array([ang_fc])
        self.ts = np.array([ts])
        self.ct = 1
        self.updated = 0
        
    def add_point(self,x_pos,y_pos,z_height,vel,ang_mo,ang_fc,ts):
        self.points = np.concatenate((self.points,[[x_pos,y_pos]]),axis=0)
        self.points_px = np.concatenate((self.points_px,[[int((x_pos - (-1*60000))/50) , int((20000 - (y_pos))/50)]]),axis=0)
        self.x = int((x_pos - (-1*60000))/50)
        self.y = int((20000 - (y_pos))/50)
        self.z_height = np.concatenate((self.z_height,[z_height]),axis=0)
        self.vel = np.concatenate((self.vel,[vel]),axis=0)
        self.ang_mo = np.concatenate((self.ang_mo,[ang_mo]),axis=0)
        self.ang_fc = np.concatenate((self.ang_fc,[ang_fc]),axis=0)
        self.ts = np.concatenate((self.ts,[ts]),axis=0)
        self.ct = self.ct+1
        self.updated = 1
        
    def degrees(self):
        if(self.ang_mo[-1]>0):
            ang1 = round(math.degrees(self.ang_mo[-1]))
        else:
            ang1 = 360-round(math.degrees(abs(self.ang_mo[-1])))
        ang1 = ang1+90 
        return ang1    
