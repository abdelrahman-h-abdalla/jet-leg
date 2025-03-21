# -*- coding: utf-8 -*-
"""
Created on Tue Oct 30 15:18:59 2018

@author: Romeo Orsolino
"""

import numpy as np
# from scipy.spatial import ConvexHull 
from jet_leg.computational_geometry.math_tools import Math

class Geometry:
    
    def clockwise_sort(self, polygon):
        ''' we assume here that the vertices are on the rows'''

        vertices_number = np.size(polygon,0)
        angle = np.zeros((1,vertices_number))
        centroidX = np.sum(polygon[:,0])
        centroidX = centroidX/float(vertices_number)
        centroidY = np.sum(polygon[:,1])
        centroidY = centroidY/float(vertices_number)
        
        for j in range(0,vertices_number):
            angle[0,j] = np.arctan2(polygon[j,0] - centroidX, polygon[j,1] - centroidY)
        
        index = np.array(np.argsort(angle))
        sorted_vertices = np.zeros((vertices_number,2))
        for j in range(0,vertices_number):
            sorted_vertices[j,:] = polygon[index[0,j],:]       
        
        # adding an extra point to close the polytop (last point equal to the first)
        sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0,:]])
        
        return sorted_vertices