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
sc.setRoi(np.float32([[1076, 254]]), 700, 110, 42.7, show = False)

'''
sc.readNextFrame()
sc.renderScene(15000) # Let the window live for a certain time in millisecs
'''

try:
    while sc.readNextRow(): 
        pass#sc.renderScene(waitTime = 1)
except KeyboardInterrupt:
    sc.deallocate()
except:
    sc.deallocate()
    raise

