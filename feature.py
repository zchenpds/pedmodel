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
        
        self.phi_sam = np.zeros(1,10)#Whether each SAM bin has velocity or not
        self.phi_sam_vel = np.zeros(1,10)#SAM bin velocity
        self.phi_sam_vel_bin = np.zeros(1,30)#SAM velocity bins
        self.phi_rel_vel_ang = np.zeros(1,3)#relative velocity bin
        self.phi_rel_vel_mag = np.zeros(1,3)#relative velocity magnitude
        self.phi_rel_vel_bin = np.zeros(1,9)#relative velocity bins
        
        
        pos = state.pos#position of the current pedestrian (numpy 1X2 array in mm)
        
        for id, neighborState in State.neighborStates.items():
            neighborPos = neighborState.pos#position of the neighbors (numpy 1X2 array in mm)
            self.within_radius+=self._density(pos,neighborPos)
            self.sam (state, neighborState)#generate SAM velocities
            self.rel_pos = self.rel_pos_cal(pos,neighborPos)#calculate relative position
            self.rel_vel = self.rel_vel_cal(state, neighborState)#calculate relative position
            self.ang_bet = self._angle_between(self.rel_pos,self.rel_vel)#angle between position vector and velocity vector
            self._rel_vel_val(self.ang_bet,self.rel_vel)


        self.phi_d = self._density_descretize()#generate the final denisty feature
        self.sam_vel_des() #generate the final sam feature
        self._rel_vel_dis() #generate the final relative velocity feature
        return self.phi_d,self.phi_sam,self.phi_sam_vel_bin,self.phi_rel_vel_ang,self.phi_rel_vel_bin

    def _rel_vel_dis(self):#function to discretize relative velocity
        
        if self.phi_rel_vel_ang[0]==1:
            if self.phi_rel_vel_mag[0]<=0.015:
                self.phi_rel_vel_bin[0] = 1
            elif self.phi_rel_vel_mag[0]<=0.025:
                self.phi_rel_vel_bin[1] = 1
            elif self.phi_rel_vel_mag[0]>0.025:
                self.phi_rel_vel_bin[2] = 1

        if self.phi_rel_vel_ang[1]==1:
            if self.phi_rel_vel_mag[1]<=0.015:
                self.phi_rel_vel_bin[3] = 1
            elif self.phi_rel_vel_mag[1]<=0.025:
                self.phi_rel_vel_bin[4] = 1
            elif self.phi_rel_vel_mag[1]>0.025:
                self.phi_rel_vel_bin[5] = 1

        if self.phi_rel_vel_ang[2]==1:
            if self.phi_rel_vel_mag[2]<=0.015:
                self.phi_rel_vel_bin[6] = 1
            elif self.phi_rel_vel_mag[2]<=0.025:
                self.phi_rel_vel_bin[7] = 1
            elif self.phi_rel_vel_mag[2]>0.025:
                self.phi_rel_vel_bin[8] = 1
                
    def _rel_vel_val(self, ang_bet,rel_vel):#function to calculate relative angle bin
        if ((ang_bet<=3*math.pi/4)and(ang_bet>=-3*math.pi/4)):
            self.phi_rel_vel_ang[0]=1
            if self.phi_rel_vel_mag[0]==0:
                self.phi_rel_vel_mag[0]=math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2))
            else:
                self.phi_rel_vel_mag[0]=(self.phi_rel_vel_mag[0]+math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2)))/2
        elif (((ang_bet>math.pi/4)and(ang_bet<3*math.pi/4))or((ang_bet<-1*math.pi/4)and(ang_bet>-3*math.pi/4))):
            self.phi_rel_vel_ang[1]=1
            if self.phi_rel_vel_mag[1]==0:
                self.phi_rel_vel_mag[1]=math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2))
            else:
                self.phi_rel_vel_mag[1]=(self.phi_rel_vel_mag[1]+math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2)))/2
                
        elif ((ang_bet<=math.pi/4)and(ang_bet>=-1*math.pi/4)):
            self.phi_rel_vel_ang[2]=1
            if self.phi_rel_vel_mag[2]==0:
                self.phi_rel_vel_mag[2]=math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2))
            else:
                self.phi_rel_vel_mag[2]=(self.phi_rel_vel_mag[2]+math.sqrt(math.pow(rel_vel[0],2)+math.pow(rel_vel[1],2)))/2


    def _rel_pos_cal(self, pos, ne_pos):#function to calculate relative position vector
        return [pos[0]-ne_pos[0],pos[1]-ne_pos[1]]

    def _rel_vel_cal(self, state, neighborState):#function to calculate relative velocity vector
        return [state.vel*math.cos(state.angM)-neighborState.vel*math.cos(neighborState.angM),state.vel*math.sin(state.angM)-neighborState.vel*math.sin(neighborState.angM)]
        
    def _density(self, pos, ne_pos):#function to check if a neighbor is within the desired radius or not
        dis = math.sqrt(math.pow(pos[0]-ne_pos[0],2)+math.pow(pos[1]-ne_pos[1],2))
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
        
    def _sam (self, state_ped, state_neighbor):#function to calculate the average velocity of each SAM feature
        
        dir_ped = (math.cos(state_ped.angM),math.sin(state_ped.angM))
        dir_neb = (math.cos(state_neighbor.angM),math.sin(state_neighbor.angM))
        ang = self._angle_between(dir_ped, dir_neb)
        dis = math.sqrt(math.pow(state_ped.pos[0]-state_neighbor.pos[0],2)+math.pow(state_ped.pos[1]-state_neighbor.pos[1],2))

        if ((ang<=math.pi/3) and (ang>=-1*math.pi/3)and (dis<=self.close_dis)):#bin 1
            self.phi_sam[0]=1
            if self.phi_sam_vel[0]==0:
                self.phi_sam_vel[0] = state_neighbor.vel
            else:
                self.phi_sam_vel[0] = (self.phi_sam_vel[0]+state_neighbor.vel)/2

        elif ((ang>math.pi/3) and (ang<=2*math.pi/3)and (dis<=self.close_dis)):#bin 2
            self.phi_sam[1]=1
            if self.phi_sam_vel[1]==0:
                self.phi_sam_vel[1] = state_neighbor.vel
            else:
                self.phi_sam_vel[1] = (self.phi_sam_vel[1]+state_neighbor.vel)/2
                
        elif ((ang>2*math.pi/3) and (ang<-1*math.pi*2/3)and (dis<=self.close_dis)):#bin 3
            self.phi_sam[2]=1
            if self.phi_sam_vel[2]==0:
                self.phi_sam_vel[2] = state_neighbor.vel
            else:
                self.phi_sam_vel[2] = (self.phi_sam_vel[2]+state_neighbor.vel)/2

        elif ((ang>=-1*math.pi*2/3) and (ang<-1*math.pi/3) and (dis<=self.close_dis)):#bin 4
            self.phi_sam[3]=1
            if self.phi_sam_vel[3]==0:
                self.phi_sam_vel[3] = state_neighbor.vel
            else:
                self.phi_sam_vel[3] = (self.phi_sam_vel[3]+state_neighbor.vel)/2
                
        elif ((ang<=math.pi/3) and (ang>=-1*math.pi/3) and (dis<=self.radius)):#bin 5
            self.phi_sam[4]=1
            if self.phi_sam_vel[4]==0:
                self.phi_sam_vel[4] = state_neighbor.vel
            else:
                self.phi_sam_vel[4] = (self.phi_sam_vel[4]+state_neighbor.vel)/2
                
        elif ((ang<-1*math.pi/3) and (ang>=-1*math.pi/2)and (dis<=self.radius)):#bin 6
            self.phi_sam[5]=1
            if self.phi_sam_vel[5]==0:
                self.phi_sam_vel[5] = state_neighbor.vel
            else:
                self.phi_sam_vel[5] = (self.phi_sam_vel[5]+state_neighbor.vel)/2
                
        elif ((ang<-1*math.pi/2) and (ang>=math.pi*-1*2/3)and (dis<=self.radius)):#bin 7
            self.phi_sam[6]=1
            if self.phi_sam_vel[6]==0:
                self.phi_sam_vel[6] = state_neighbor.vel
            else:
                self.phi_sam_vel[6] = (self.phi_sam_vel[6]+state_neighbor.vel)/2
                
        elif ((ang>2*math.pi/3) and (ang<-1*math.pi*2/3)and (dis<=self.radius)):#bin 8
            self.phi_sam[7]=1
            if self.phi_sam_vel[7]==0:
                self.phi_sam_vel[7] = state_neighbor.vel
            else:
                self.phi_sam_vel[7] = (self.phi_sam_vel[7]+state_neighbor.vel)/2
                
        elif ((ang>=math.pi/2) and (ang<=2*math.pi/3)and (dis<=self.radius)):#bin 9
            self.phi_sam[8]=1
            if self.phi_sam_vel[8]==0:
                self.phi_sam_vel[8] = state_neighbor.vel
            else:
                self.phi_sam_vel[8] = (self.phi_sam_vel[8]+state_neighbor.vel)/2
                
        elif ((ang>math.pi/3) and (ang<math.pi/2)and (dis<=self.radius)):#bin 10
            self.phi_sam[9]=1
            if self.phi_sam_vel[9]==0:
                self.phi_sam_vel[9] = state_neighbor.vel
            else:
                self.phi_sam_vel[9] = (self.phi_sam_vel[9]+state_neighbor.vel)/2
            
    def _angle_between(self,v1, v2):
        
        """ Returns the angle in radians between vectors 'v1' and 'v2'::
        """

        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    def _sam_vel_des(self):#function to discretize the sam velocity feature bins
         """ Turns the calculated average velocities of each SAM feature to a discrete
         """      
         
         if self.phi_sam_vel[0]<=1:
            self.phi_sam_vel_bin[0] = 1
         elif self.phi_sam_vel[0]<=2:
            self.phi_sam_vel_bin[1] = 1
         elif self.phi_sam_vel[0]>2:
            self.phi_sam_vel_bin[2] = 1

         if self.phi_sam_vel[1]<=1:
            self.phi_sam_vel_bin[3] = 1
         elif self.phi_sam_vel[1]<=2:
            self.phi_sam_vel_bin[4] = 1
         elif self.phi_sam_vel[1]>2:
            self.phi_sam_vel_bin[5] = 1
            
         if self.phi_sam_vel[2]<=1:
            self.phi_sam_vel_bin[6] = 1
         elif self.phi_sam_vel[3]<=2:
            self.phi_sam_vel_bin[7] = 1
         elif self.phi_sam_vel[3]>2:
            self.phi_sam_vel_bin[8] = 1

         if self.phi_sam_vel[3]<=1:
            self.phi_sam_vel_bin[9] = 1
         elif self.phi_sam_vel[3]<=2:
            self.phi_sam_vel_bin[10] = 1
         elif self.phi_sam_vel[3]>2:
            self.phi_sam_vel_bin[11] = 1

         if self.phi_sam_vel[4]<=1:
            self.phi_sam_vel_bin[12] = 1
         elif self.phi_sam_vel[4]<=2:
            self.phi_sam_vel_bin[13] = 1
         elif self.phi_sam_vel[4]>2:
            self.phi_sam_vel_bin[14] = 1

         if self.phi_sam_vel[5]<=1:
            self.phi_sam_vel_bin[15] = 1
         elif self.phi_sam_vel[5]<=2:
            self.phi_sam_vel_bin[16] = 1
         elif self.phi_sam_vel[5]>2:
            self.phi_sam_vel_bin[17] = 1

         if self.phi_sam_vel[6]<=1:
            self.phi_sam_vel_bin[18] = 1
         elif self.phi_sam_vel[6]<=2:
            self.phi_sam_vel_bin[19] = 1
         elif self.phi_sam_vel[6]>2:
            self.phi_sam_vel_bin[20] = 1

         if self.phi_sam_vel[7]<=1:
            self.phi_sam_vel_bin[21] = 1
         elif self.phi_sam_vel[7]<=2:
            self.phi_sam_vel_bin[22] = 1
         elif self.phi_sam_vel[7]>2:
            self.phi_sam_vel_bin[23] = 1

         if self.phi_sam_vel[8]<=1:
            self.phi_sam_vel_bin[24] = 1
         elif self.phi_sam_vel[8]<=2:
            self.phi_sam_vel_bin[25] = 1
         elif self.phi_sam_vel[8]>2:
            self.phi_sam_vel_bin[26] = 1

         if self.phi_sam_vel[9]<=1:
            self.phi_sam_vel_bin[27] = 1
         elif self.phi_sam_vel[9]<=2:
            self.phi_sam_vel_bin[28] = 1
         elif self.phi_sam_vel[9]>2:
            self.phi_sam_vel_bin[29] = 1            
            