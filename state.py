# -*- coding: utf-8 -*-
"""
Created on Mon Nov  6 15:01:53 2017

@author: cz
"""
import numpy as np

class State:
    def __init__(self, dataEntry, timestep):
        ''' Constructor
        '''
        self.data = dataEntry # not in use for now
        self._timestep = timestep
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
        
    def updateNeighbors(self, curPeds):
        neighbors = curPeds.copy()
        del neighbors[self.id]
        self._neighbors = neighbors
        # To-do: update neighborStates
    