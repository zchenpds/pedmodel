#file -- Pedestrian.py --
"""
Author: Mfahad
"""
import numpy as np
from interpolated_trajectory import Interpolated
from feature import Feature
from state import State
class Pedestrian():
    def __init__(self, dataEntry, scene):
        self.inter_traj = np.array([])
        self.inter_traj.shape = (0,2)
        self.points = np.array([]) # obsolete
        self.points.shape = (0,2) # obsolete
        
        self.states = dict() # indexed by timeStep
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
        
        '''
        Calculate pixels to next pixel
        '''
        
        # Update current time
        self.time = self.data.loc[0, 'time_stamp']
        self.lag = self._scene.time - self.time
        print(self.lag)
        
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
        self.states[timestep] = State(dataEntry, timestep)
        
        # Update current position # obsolete
        self.pos = np.array([[dataEntry.loc[0, 'x'], # obsolete
                             dataEntry.loc[0, 'y']]]) # obsolete
    
        #print(self.pos.shape)
        
        # Update properties for visualization
        self._xPix = int(self.x() / 50)
        self._yPix = int(self.y() / 50)
        self._rPix = int(self.radius/50)
        
        # Update trajectories, each denoted by an n-by-2 array 
        self.points = np.concatenate((self.points, self.pos), axis = 0) 
        
        # Generate interpolated trajectory
        self.inter_traj = np.concatenate((self.inter_traj, np.array([[self._xPix,
                         self._yPix]])), axis = 0)

        #print self.inter_traj.shape
        #interploate the trajectory between last position and current position
        #self.inter_traj = np.concatenate(Interpolated(self.inter_traj[len(self.inter_traj)-2],self.inter_traj[len(self.inter_traj)-2]
        #,self.inter_traj[len(self.inter_traj)-1],self.inter_traj[len(self.inter_traj)-1]))


        # Update other states of the pedestrian
        self.zHeight = dataEntry.loc[0, 'z'] # obsolete
        self.vel = dataEntry.loc[0, 'vel'] # obsolete
        self.angM = dataEntry.loc[0, 'ang_m'] # obsolete
        self.angF = dataEntry.loc[0, 'ang_f'] # obsolete
        
    def x(self):
        return self.pos[0][0]
    
    def y(self):
        return self.pos[0][1]
    
    

