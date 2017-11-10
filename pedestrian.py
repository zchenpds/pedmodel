#file -- Pedestrian.py --
"""
Author: Mfahad
"""
import numpy as np
import cv2
from interpolated_trajectory import Interpolated
from feature import Feature
from state import State
from collections import OrderedDict
class Pedestrian():
    def __init__(self, dataEntry, scene):
        self.inter_traj = np.array([])
        self.inter_traj.shape = (0,2)
        self.points = np.array([]) # obsolete
        self.points.shape = (0,2) # obsolete
        
        self.states = OrderedDict() # indexed by timeStep
        # self.interStates = OrderedDict() # indexed by timeStep, has one fewer element than states
        self.features = dict() # indexed by timeStep
        
        self.radius = 200 # pedestrian radius in millimeters
        self._scene = scene
        self._update(dataEntry)
        #self.peds = scene.peds
        
        
    def update(self, dataEntry):
        self._update(dataEntry)
        
    def calculate(self, curPeds):
        ''' 
        To-dos:
            2. Feature calculation
        '''
        
        # Update current time
        self.time = self._scene.time
        self.lag = self._scene.time - self.time
        # print(self.lag)
        
        timestep = self._scene.getTimestep()
        
        # Update neighbors in state
        self.states[timestep].updateNeighbors(curPeds)
        
        # Update features
        state = self.states[timestep]
        self.features[timestep] = Feature(state, timestep)
            
            
    def _update(self, dataEntry):
        ''' Update self
        '''
        self.data = dataEntry # obsolete
        self.id = dataEntry.loc[0, 'ped_id']
        
        # Update states
        timestep = self._scene.getTimestep()
        self.states[timestep] = State(dataEntry, self)
        
        # Update interStates
        if (timestep - 1) in self.states:
            self.states[timestep - 1].updateInterState(self.states[timestep])
        #self.interStates[timestep]
        
        # Update current position # obsolete
        #self.pos = np.array([[dataEntry.loc[0, 'x'], # obsolete
        #                     dataEntry.loc[0, 'y']]]) # obsolete
    
        #print(self.pos.shape)
        
        # Update properties for visualization
        #self._xPix = int(self.x() / 50)
        #self._yPix = int(self.y() / 50)
        self._rPix = int(self.radius/50)
        
        # Update trajectories, each denoted by an n-by-2 array 
        # self.points = np.concatenate((self.points, self.pos), axis = 0) 
        
        # Generate interpolated trajectory
        self.inter_traj = np.concatenate(
                (self.inter_traj, 
                 np.array([[self.xPix(), self.yPix()]])), 
                axis = 0)

        #print self.inter_traj.shape
        #interploate the trajectory between last position and current position
        #self.inter_traj = np.concatenate(Interpolated(self.inter_traj[len(self.inter_traj)-2],self.inter_traj[len(self.inter_traj)-2]
        #,self.inter_traj[len(self.inter_traj)-1],self.inter_traj[len(self.inter_traj)-1]))


        # Update other states of the pedestrian
        #self.zHeight = dataEntry.loc[0, 'z'] # obsolete
        #self.vel = dataEntry.loc[0, 'vel'] # obsolete
        #self.angM = dataEntry.loc[0, 'ang_m'] # obsolete
        #self.angF = dataEntry.loc[0, 'ang_f'] # obsolete
        
        
    def draw(self, image):
        timestep = self._scene.getTimestep()
        if timestep in self.states:
            cv2.circle(image, 
                       center = (self.xPix(timestep), self.yPix(timestep)), 
                       radius = self._rPix, 
                       color = (0, 255, 0), thickness = -1)
        # Draw trajectories
        for timestep, state in self.states.items():
            if state.interState is not None:
                state.interState.draw(image)
        
    def x(self, timestep = -1):
        if timestep == -1:
            timestep = self._scene.getTimestep()
        return self.states[timestep].x()
    
    def y(self, timestep = -1):
        if timestep == -1:
            timestep = self._scene.getTimestep()
        return self.states[timestep].y()
    
    def xPix(self, timestep = -1):
        return int(self.x(timestep) / 50)
    
    def yPix(self, timestep = -1):
        return int(self.y(timestep) / 50)

