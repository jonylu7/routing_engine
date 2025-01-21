
import json
from FindPath import *

import os, sys
dir_path = os.path.dirname(os.path.realpath(__file__))
parent_dir_path = os.path.abspath(os.path.join(dir_path, os.pardir))

sys.path.insert(0, parent_dir_path)
import routing_agent.WaypointGraph

def generateSolution(pathMatrix, costMatrixSol):
    solutionPath=[]
    for i in range(len(costMatrixSol)-1):
        solutionPath+=(pathMatrix[costMatrixSol[i]][costMatrixSol[i+1]])[:-1]
    return solutionPath

def convertWaypointIndexsToLocations(waypointIndexs,nodeLocations):
    nodeLocations=[]
    for i in waypointIndexs:
        nodeLocations.append(nodeLocations[i])
    return nodeLocations

def generateMatrix(offsets,edges,weights):
    graph=FindPath.convertFromCSRToDijGraph(offsets,edges,weights)
    costmatrix,pathmatrix=FindPath.findAllShortestPath(graph)
    return costmatrix,pathmatrix



