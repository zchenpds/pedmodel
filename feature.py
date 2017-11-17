# -*- coding: utf-8 -*-
"""

@author: mfahad
"""

from state import State
import math
import numpy as np

class Feature():
    def __init__(self, state, timestep):
        ''' construct the feature vector from a state object
        state: self state
        '''

        self.radius = 5000 #radius of influence around pedestrian
        self.close_dis = 3000
        self.vel_thresh1 = 500
        self.vel_thresh2 = 1000
        self.vel_thresh3 = 1500
        self.phi_sam = np.zeros((10,))#Whether each SAM bin has velocity or not
        self.phi_sam_vel = np.zeros((2,10))#SAM bin velocity
        self.phi_sam_vel_bin = np.zeros((60,))#SAM velocity bins
        self.within_radius = 0#number of pedestrians within self.radius
        
        
        pos = state.pos#position of the current pedestrian (numpy 1X2 array in mm)
        
        for id, neighborState in state.neighborStates.items():
            neighborPos = neighborState.pos#position of the neighbors (numpy 1X2 array in mm)
            self.within_radius+=self._density(pos,neighborPos)
            self._sam(state, neighborState)#generate SAM velocities


        self.phi_d = self._density_descretize()#generate the final denisty feature
        self._sam_vel_des(state) #generate the final sam feature
        #return self.phi_d,self.phi_sam,self.phi_sam_vel_bin,self.phi_rel_vel_ang,self.phi_rel_vel_bin
        
    def _density(self, pos, ne_pos):#function to check if a neighbor is within the desired radius or not

        dis = math.sqrt(math.pow(pos[0,0]-ne_pos[0,0],2)+math.pow(pos[0,1]-ne_pos[0,1],2))#Calculate the distance from neighboring pedestrian. 
        if dis<self.radius:
            return 1
        else:
            return 0
        
    def _density_descretize(self):#function to discretize the density

        if self.within_radius<=2:
            return [1,0,0]
        
        elif (self.within_radius>2 and self.within_radius<=5):
            return [0,1,0]
        
        elif (self.within_radius>5):
            return [0,0,1]
        
    def _sam(self, state_ped, state_neighbor):#function to calculate the average velocity of each SAM feature
        
        dir_ped = (math.cos(state_ped.angM),math.sin(state_ped.angM))
        dir_neb = (math.cos(state_neighbor.angM),math.sin(state_neighbor.angM))
        ang = self._angle_between(dir_ped, dir_neb)
        dis = math.sqrt(math.pow(state_ped.pos[0,0]-state_neighbor.pos[0,0],2)+math.pow(state_ped.pos[0,1]-state_neighbor.pos[0,1],2))

        if ((ang<=math.pi/4) and (ang>=-1*math.pi/4)and (dis<=self.close_dis)):#bin 1
            self.phi_sam[0]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,0],2)+math.pow(self.phi_sam_vel[1,0],2))==0:
                self.phi_sam_vel[0,0] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,0] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,0] = (self.phi_sam_vel[0,0]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,0] = (self.phi_sam_vel[1,0]+state_neighbor.vel*math.sin(state_neighbor.angM))/2

        elif ((ang>math.pi/4) and (ang<=3*math.pi/4)and (dis<=self.close_dis)):#bin 2
            self.phi_sam[1]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,1],2)+math.pow(self.phi_sam_vel[1,1],2))==0:
                self.phi_sam_vel[0,1] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,1] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,1] = (self.phi_sam_vel[0,1]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,1] = (self.phi_sam_vel[1,1]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang>3*math.pi/4) and (ang<=-3*math.pi/4)and (dis<=self.close_dis)):#bin 3
            self.phi_sam[2]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,2],2)+math.pow(self.phi_sam_vel[1,2],2))==0:
                self.phi_sam_vel[0,2] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,2] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,2] = (self.phi_sam_vel[0,2]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,2] = (self.phi_sam_vel[1,2]+state_neighbor.vel*math.sin(state_neighbor.angM))/2

        elif ((ang>=-3*math.pi/4) and (ang<-1*math.pi/4) and (dis<=self.close_dis)):#bin 4
            self.phi_sam[3]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,3],2)+math.pow(self.phi_sam_vel[1,3],2))==0:
                self.phi_sam_vel[0,3] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,3] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,3] = (self.phi_sam_vel[0,3]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,3] = (self.phi_sam_vel[1,3]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang<=math.pi/4) and (ang>=-1*math.pi/4)and (dis<=self.close_dis)):#bin 5
            self.phi_sam[4]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,4],2)+math.pow(self.phi_sam_vel[1,4],2))==0:
                self.phi_sam_vel[0,4] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,4] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,4] = (self.phi_sam_vel[0,4]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,4] = (self.phi_sam_vel[1,4]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang<-1*math.pi/4) and (ang>=-1*math.pi/2)and (dis<=self.radius)):#bin 6
            self.phi_sam[5]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,5],2)+math.pow(self.phi_sam_vel[1,5],2))==0:
                self.phi_sam_vel[0,5] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,5] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,5] = (self.phi_sam_vel[0,5]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,5] = (self.phi_sam_vel[1,5]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang<-1*math.pi/2) and (ang>=math.pi*-1*3/4)and (dis<=self.radius)):#bin 7
            self.phi_sam[6]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,6],2)+math.pow(self.phi_sam_vel[1,6],2))==0:
                self.phi_sam_vel[0,6] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,6] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,6] = (self.phi_sam_vel[0,6]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,6] = (self.phi_sam_vel[1,6]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang>=3*math.pi/4) and (ang<-1*math.pi*3/4)and (dis<=self.radius)):#bin 8
            self.phi_sam[7]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,7],2)+math.pow(self.phi_sam_vel[1,7],2))==0:
                self.phi_sam_vel[0,7] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,7] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,7] = (self.phi_sam_vel[0,7]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,7] = (self.phi_sam_vel[1,7]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang>math.pi/2) and (ang<3*math.pi/4)and (dis<=self.radius)):#bin 9
            self.phi_sam[8]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,8],2)+math.pow(self.phi_sam_vel[1,8],2))==0:
                self.phi_sam_vel[0,8] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,8] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,8] = (self.phi_sam_vel[0,8]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,8] = (self.phi_sam_vel[1,8]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
                
        elif ((ang>math.pi/4) and (ang<=math.pi/2)and (dis<=self.radius)):#bin 10
            self.phi_sam[9]=1
            if math.sqrt(math.pow(self.phi_sam_vel[0,9],2)+math.pow(self.phi_sam_vel[1,9],2))==0:
                self.phi_sam_vel[0,9] = state_neighbor.vel*math.cos(state_neighbor.angM)
                self.phi_sam_vel[1,9] = state_neighbor.vel*math.sin(state_neighbor.angM)
            else:
                self.phi_sam_vel[0,9] = (self.phi_sam_vel[0,9]+state_neighbor.vel*math.cos(state_neighbor.angM))/2
                self.phi_sam_vel[1,9] = (self.phi_sam_vel[1,9]+state_neighbor.vel*math.sin(state_neighbor.angM))/2
            
    def _angle_between(self,v1, v2):
        
        """ Returns the angle in radians between vectors 'v1' and 'v2'::
        """

        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    def _sam_vel_des(self,state):#function to discretize the sam velocity feature bins
         """ Turns the calculated average velocities of each SAM feature to a discrete
         """      
         dir_ped = (state.vel*math.cos(state.angM),state.vel*math.sin(state.angM))
         
         if self.phi_sam[0]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,0],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,0],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,0],self.phi_sam_vel[1,0]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[0] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[1] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[2] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[3] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[4] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[5] = 1   
                 
         if self.phi_sam[1]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,1],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,1],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,1],self.phi_sam_vel[1,1]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[6] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[7] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[8] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[9] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[10] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[11] = 1                   

         if self.phi_sam[2]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,2],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,2],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,2],self.phi_sam_vel[1,2]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[12] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[13] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[14] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[15] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[16] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[17] = 1  
                 
         if self.phi_sam[3]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,3],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,3],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,3],self.phi_sam_vel[1,3]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[18] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[19] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[20] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[21] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[22] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[23] = 1  
                 
         if self.phi_sam[4]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,4],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,4],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,4],self.phi_sam_vel[1,4]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[24] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[25] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[26] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[27] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[28] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[29] = 1 
 
         if self.phi_sam[5]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,5],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,5],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,5],self.phi_sam_vel[1,5]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[30] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[31] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[32] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[33] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[34] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[35] = 1 

         if self.phi_sam[6]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,6],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,6],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,6],self.phi_sam_vel[1,6]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[36] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[37] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[38] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[39] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[40] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[41] = 1 

         if self.phi_sam[7]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,7],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,7],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,7],self.phi_sam_vel[1,7]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[42] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[43] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[44] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[45] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[46] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[47] = 1 
                
         if self.phi_sam[8]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,8],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,8],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,8],self.phi_sam_vel[1,8]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[48] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[49] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[50] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[51] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[52] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[53] = 1 

         if self.phi_sam[9]>=1:
             vel = (math.pow(dir_ped[0]-self.phi_sam_vel[0,9],2),math.pow(dir_ped[1]-self.phi_sam_vel[1,9],2))
             ang_bet = self._angle_between(dir_ped,(self.phi_sam_vel[0,9],self.phi_sam_vel[1,9]))
             if vel<=self.vel_thresh1:
                 self.phi_sam_vel_bin[54] = 1
             elif vel<=self.vel_thresh2:
                 self.phi_sam_vel_bin[55] = 1
             elif vel<=self.vel_thresh3:
                 self.phi_sam_vel_bin[56] = 1
             if ((ang_bet >3*math.pi/4) and ang_bet<=(-3*math.pi/4)):
                 self.phi_sam_vel_bin[57] = 1
             elif (((ang_bet <=3*math.pi/4) and (ang_bet>(math.pi/4)))or((ang_bet >-3*math.pi/4) and (ang_bet<=(-1*math.pi/4)))):
                 self.phi_sam_vel_bin[58] = 1         
             elif ((ang_bet <math.pi/4) and ang_bet>(-1*math.pi/4)):
                 self.phi_sam_vel_bin[59] = 1 
                 