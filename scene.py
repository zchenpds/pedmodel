# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 12:54:12 2017

@author: cz
"""

import cv2
import pandas as pd
import numpy as np
import math
from collections import OrderedDict

from pedestrian import Pedestrian

class Scene():
    def __init__(self):
        """ Initializer
        """
        self._ind = 0
        self._timestep = -1 # indicating that no data is available yet
        self._headers = ["time_stamp","ped_id","x","y","z","vel","ang_m","ang_f"]
        # ped_list
        self.peds = dict()
        self.pedList = OrderedDict()
        self._imageIn = None
        self._imageOut = None
        self._M = None # affine transform in millimeters
        self._wRoi = None # ROI width in millimeters
        self._hRoi = None # ROI height in millimeters
        
        # Exit handler (This fixes the window freezing bug)
        # atexit.register(self.deallocate)
        
    def setRoi(self, oPix, wPix, hPix, deg, show = False):
        ''' Set region of interest to a rectagular one
        oPix: a 2-tuple denoting the origin (in pixels)
        wPix: a scalar denoting width (in pixels)
        hPix: a scalar denoting height (in pixels)
        deg: an angle by which the rectangle is to be rotated around the origin
        '''
        # Find the affine transform for scene image (in pixels)
        vec1 = np.float32([[math.cos(math.radians(deg)), 
                            math.sin(math.radians(deg))]])
        vec2 = np.float32([[math.sin(math.radians(deg)), 
                            -math.cos(math.radians(deg))]])
        p1Pix = oPix + vec1 * wPix
        p2Pix = oPix + vec2 * hPix
        if show:
            p3Pix = oPix + vec1 * wPix + vec2 * hPix # for debug
            cv2.line(self._imageIn, tuple(oPix[0]), tuple(p3Pix[0]), (0, 0, 255))
            cv2.line(self._imageIn, tuple(p1Pix[0]), tuple(p2Pix[0]), (0, 0, 255))
            cv2.line(self._imageIn, tuple(oPix[0]), tuple(p1Pix[0]), (0, 0, 255))
            cv2.line(self._imageIn, tuple(oPix[0]), tuple(p2Pix[0]), (0, 0, 255))
            cv2.line(self._imageIn, tuple(p3Pix[0]), tuple(p2Pix[0]), (0, 0, 255))
            cv2.line(self._imageIn, tuple(p3Pix[0]), tuple(p1Pix[0]), (0, 0, 255))
            cv2.imshow('img', self._imageIn)
        pts1 = np.concatenate((oPix, p1Pix, p2Pix))
        pts2 = np.float32([[0, 0], [wPix, 0], [0, hPix]])
        M = cv2.getAffineTransform(pts1, pts2) 
        cols, rows, ch = self._imageIn.shape
        img2 = cv2.warpAffine(self._imageIn, M, (cols,rows))
        # Crop scene image
        self._imageIn = img2[0:hPix, 0:wPix] 
        
        # Find the affine transform for csv data (in millimeters)
        self._wRoi = wPix*50
        self._hRoi = hPix*50
        o = self._pix2mm(oPix)
        p1 = self._pix2mm(p1Pix)
        p2 = self._pix2mm(p2Pix)
        pts1 = np.concatenate((o, p1, p2))
        pts2 = np.float32([[0, 0], [self._wRoi, 0], [0, self._hRoi]])
        self._M = cv2.getAffineTransform(pts1, pts2) # affine transform
        
    def _pix2mm(self, p):
        x, y = tuple(p[0])
        x = x*50 - 60000
        y = 20000 - y*50
        return np.float32([[x, y]])
    
    def _mm2pix(self, p):
        x, y = tuple(p[0])
        x = (x + 60000) / 50
        y = (20000 - y) / 50
        return np.float32([[x, y]])
    
    def _transformAndCrop(self, dataEntry):
        ''' Can only handle the first row of a dataFrame
        '''
        if self._M is None:
            return True
        x = dataEntry.loc[0, 'x']
        y = dataEntry.loc[0, 'y']
        p = np.float32([[x, y]])
        p = np.dot(self._M[:, 0:2], p.transpose())  + self._M[:, 2:]
        # print(p)
        x = p[0]
        y = p[1]
        dataEntry.loc[0, 'x'] = x
        dataEntry.loc[0, 'y'] = y
        if x > 0 and x < self._wRoi and y > 0 and y < self._hRoi:
            return True # indicates this pedestrian is in ROI
        else:
            return False # indicates this pedestrian is out of ROI
        
        
    def addMap(self, fileName):
        """ specify the path to an image file as the scene map
        """
        self._imageIn = cv2.imread(fileName, -1)
            
        
    def openCsv(self, fileName):
        """ specify the path to a csv file where pedestrian data is stored.
        """
        self._csvFileName = fileName
        
    def readNextFrame(self):
        """ read in pedestrian data of the next time step.
        """
        self._timestep = self._timestep + 1
        timestep = self._timestep
        # read in one row of data
        dataEntry = pd.read_csv(self._csvFileName, 
                                          names = self._headers, 
                                          skiprows = self._ind, nrows = 1)
        if timestep == 0:
            self.timebase = dataEntry['time_stamp'][0]
        
        # self.time is relative to the first timestamp in csv
        self.time = dataEntry['time_stamp'][0] - self.timebase
        print('Time: ' + str(self.time) + 's')
        
        # Loop over rows of data from the csv file
        pedNum = 0 # Num of peds in this frame
        curPeds = dict() # List of peds in this frame
        while True:
            # add the new entry to curPeds
            id = dataEntry['ped_id'][0]
            # Check if this pedestrian is in the ROI
            # In _transformAndCrop, dataEntry will be changed
            isInRoi = self._transformAndCrop(dataEntry) #########################
            if id in self.peds:
                # This pedestrian already exists in peds, so update it
                self.peds[id].update(dataEntry)
                self.peds[id].states[timestep]._isInRoi = isInRoi
            else:
                # Otherwise, a new pedestrian needs to be constructed
                self.peds[id] = Pedestrian(dataEntry, self)
                self.peds[id].states[timestep]._isInRoi = isInRoi
            
            curPeds[id] = self.peds[id]
            
            pedNum = pedNum + 1
            dataEntry_1 = dataEntry
            
            # read the next row
            self._ind = self._ind + 1
            dataEntry = pd.read_csv(self._csvFileName, 
                                              names = self._headers, 
                                              skiprows = self._ind, nrows = 1)
            # If EOF is reached, jump out of the loop
            if dataEntry.empty:
                break
            # If the next row has a different timestamp, jump out of the loop
            if dataEntry_1.values[0, 0] != dataEntry.values[0, 0]:
                break
        if pedNum > 0:
            print('Num of peds: ' + str(pedNum))
            
        self.pedList[timestep] = curPeds
        # Loop over all pedestrians in curPeds
        for id in list(curPeds):
            ped = self.peds[id]
            # Calculate features
            ped.calculate(curPeds)
        
        # Determine return value
        if dataEntry.empty:
            return False # EOF has been reached
        else:
            return True
            
    def renderScene(self, timestep = -1, waitTime = 25):
        ''' Create an image object to visualize what's going on.
        '''
        if timestep == -1:
            timestep = self.getTimestep()
            
        self._imageOut = self._imageIn.copy()
        for id, ped in self.pedList[timestep].items():
            ped.draw(self._imageOut)
        cv2.imshow('image',self._imageOut)
        #cv2.imwrite('output.png',self._imageOut)
        cv2.waitKey(waitTime)

        
    def deallocate(self):
        cv2.destroyAllWindows() # Add this to fix the window freezing bug
    
    def getTimestep(self): # depreciated
        return self._timestep
        
        