#file -- Pedestrian.py --
"""
Author: Mfahad
"""
import numpy as np
import pandas as pd
import math

class Pedestrian():
    def __init__(self, dataEntry: pd.DataFrame, scene):
        self.radius = 200 # pedestrian radius
        self._scene = scene
        self._update(dataEntry)
        self.pedList = scene.pedList
        
    def update(self, dataEntry: pd.DataFrame):
        self._update(dataEntry)
        
    def calculate(self):
        ''' 
        To-dos:
            2. Feature calculation
        '''
        # Update current time
        self.time = self.data.loc[0, 'time_stamp']
        # Loop over other pedestrians
        for id, other in self.pedList.items():
            if self.id == id:
                continue
            #print(np.linalg.norm(self.pos - other.pos))
            
    def _update(self, dataEntry: pd.DataFrame):
        ''' Update attributes when self.data changes
        '''
        self.data = dataEntry
        self.id = self.data.loc[0, 'ped_id']
        # Update current position
        self.pos = np.array([self.data.loc[0, 'x'],
                              self.data.loc[0, 'y']])
        print(self.pos)
        # Update trajectories， each denoted by an n-by-2 array
        if hasattr(self, 'points'):
            self.points = np.concatenate((self.points, self.pos), axis = 0)
        else:
            self.points = self.pos
        # Update other states of the pedestrian
        self.zHeight = self.data.loc[0, 'z']
        self.vel = self.data.loc[0, 'vel']
        self.angM = self.data.loc[0, 'ang_m']
        self.angF = self.data.loc[0, 'ang_f']
        
        # Update properties for visualization
        self._xPix = int(self.x() / 50)
        self._yPix = int(self.y() / 50)
        self._rPix = int(self.radius/50)
        
    
    def x(self):
        return self.pos[0]
    
    def y(self):
        return self.pos[1]
    
    
class Features():
    pass