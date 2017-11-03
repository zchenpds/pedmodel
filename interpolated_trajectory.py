#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 17:24:00 2017

The main purpose of this class is a new data type that holds the pixels the pedestrian moves through to reach the next 
recorded location of the pedestrian.

@author: mfahad
"""

from bresenham import bresenham

class Interpolated():
    
    def __init__(self,x0,y0,x1,y1):
        
        self.traj = list(bresenham(x0,y0,x1,y1))
