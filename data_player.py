# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 13:04:07 2017

@author: cz
"""

# data_player

from scene import Scene
import numpy as np

sc = Scene()
sc.addMap('scene_map.png')
sc.openCsv('atc-20121114.csv')
#sc.openCsv('test.csv')
sc.setRoi(np.float32([[1093, 237]]), 713, 100, 42.7, show = False)
'''
sc.readNextFrame()
sc.renderScene(5000) # Let the window live for a certain time in millisecs
'''


while sc.readNextFrame(): 
    pass#sc.renderScene(250)

sc.deallocate()