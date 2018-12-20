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
#sc.addMap('fahad1.png')
#sc.openCsv('atc-20121114.csv')
#sc.openCsv('test.csv')
sc.setRoi(np.float32([[1076, 254]]), 700, 110, 42.7, show = True)
#sc.setRoi(np.float32([[0, 0]]), 1200, 2800, 90, show = False)

'''
sc.readNextFrame()
sc.renderScene(15000) # Let the window live for a certain time in millisecs
'''

#input("Press Enter to continue...")


try:
    while sc.readNextRow(): 
        if sc.time > 300:
            sc.renderScene(waitTime = 1, write = True)
            break
        else:
            sc.renderScene(waitTime = 1, write = False, show = False)
    sc.deallocate()
except KeyboardInterrupt:
    sc.deallocate()
except:
    sc.deallocate()
    raise

