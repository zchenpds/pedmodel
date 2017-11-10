# -*- coding: utf-8 -*-
"""
Created on Mon Nov  6 15:01:53 2017

@author: cz
"""
import numpy as np
from bresenham import bresenham

class State:
    def __init__(self, dataEntry, ped):
        ''' Constructor
        '''
        self.data = dataEntry # not in use for now
        self._timestep = ped._scene.getTimestep()
        self.time = ped._scene.time
        self.id = dataEntry.loc[0, 'ped_id']
        # Update current position
        self.pos = np.array([[self.data.loc[0, 'x'],
                             self.data.loc[0, 'y']]])
        self.zHeight = dataEntry.loc[0, 'z']
        self.vel = dataEntry.loc[0, 'vel']
        self.angM = dataEntry.loc[0, 'ang_m'] # not rotated yet
        self.angF = dataEntry.loc[0, 'ang_f'] # not rotated yet
        
        # Upon initialization, _neighbors will still be empty
        self.neighborStates = dict()
        # self._isInRoi will be updated by Scene
        
        self.nextState = None
        self.interState = None
        
    def updateInterState(self, nextState):
        self.nextState = nextState
        self.interState = InterState(self, nextState)
        
        
        
    def updateNeighbors(self, curPeds):
        neighbors = curPeds.copy()
        del neighbors[self.id]
        self._neighbors = neighbors
        # To-do: update neighborStates
        for id, neighbor in neighbors.items():
            self.neighborStates[id] = neighbor.states[self._timestep]
    
    def x(self):
        return self.pos[0][0]
    
    def y(self, timestep = -1):
        return self.pos[0][1]
    
    def xPix(self):
        return int(self.x() / 50)
    
    def yPix(self):
        return int(self.y() / 50)
    
    
class InterState():
    def __init__(self, state1, state2):
        #print(str(state2.xPix()) + ", " + str(state2.yPix()))
        self.traj = list(bresenham(state1.xPix(), state1.yPix(), 
                                   state2.xPix(), state2.yPix()))
    
    def draw(self, image):
        for i in range(len(self.traj)):
            #print(self.traj[i])
            if (self.traj[i][1] in range(image.shape[0]) and 
                self.traj[i][0] in range(image.shape[1])):
                image[self.traj[i][1], self.traj[i][0]] = np.uint8([0, 255, 0])
    