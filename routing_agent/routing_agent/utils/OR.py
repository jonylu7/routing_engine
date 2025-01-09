
import json
from routing_agent.utils.FindPath import *
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



