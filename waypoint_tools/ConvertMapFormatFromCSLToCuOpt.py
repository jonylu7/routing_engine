
import math
#from LoadToolkit import loadWaypointGraphData
indoor="waypointgraph_indoor"
new="waypointgraph_new_1"
outdoor="waypointgraph_outdoor"
og="waypointgraph"

import sys
sys.path.append("../routing_agent/routing_agent/utils/")
#import WaypointGraph
from ConvertDataFormat import loadJSONFile,saveJSONAt


def ConvertAutomatically(from_path,to_path):
    data=loadJSONFile(from_path)
    nodelist=list(data.keys())
    locationlist=[]
    graph={}
    localToGlobalLocation={}
    for index,nodeid in enumerate(nodelist):

        if(len(localToGlobalLocation.items())==0):
            localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]]=[0,0,0]
        
        thisMapId=getIdAndIndexByNodeId(nodeid)[0]

        edges=[]
        for eId in data[nodeid]["edges"]:

            edges.append(nodelist.index(eId))
            
            if(getIdAndIndexByNodeId(eId)[0]!=thisMapId):
                if(getIdAndIndexByNodeId(eId)[0] not in localToGlobalLocation.keys()):
                    localToGlobalLocation[getIdAndIndexByNodeId(eId)[0]]=[
                        data[nodeid]["local_location"][0]-data[eId]["local_location"][0]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][0]
                        ,data[nodeid]["local_location"][1]-data[eId]["local_location"][1]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][1]
                        ,data[nodeid]["local_location"][2]-data[eId]["local_location"][2]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][2]
                    ]
                #adjust
        print(localToGlobalLocation)
        locationlist.append([data[nodeid]["local_location"][0]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][0]
                            ,data[nodeid]["local_location"][1]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][1]
                            ,data[nodeid]["local_location"][2]+localToGlobalLocation[getIdAndIndexByNodeId(nodeid)[0]][2]]
                            )
        if(str(index) not in graph.keys()):
            graph[str(index)]={}
        graph[str(index)]["edges"]=edges
       
        print(nodeid)
    print(locationlist)
    print(graph)
    
    outputdata={"node_locations":locationlist,"graph":graph}
    saveJSONAt(outputdata,to_path)

def getEntryPoints(data):
    entrypoints={}
    nodelist=list(data.keys())
    for index,nodeid in enumerate(nodelist):
        for eId in data[nodeid]["edges"]:
            if(getMapIdByNodeId(eId)!=getMapIdByNodeId(nodeid)):
                if(nodeid not in list(entrypoints.keys())):
                    entrypoints[nodeid]=eId
            
    return entrypoints

                
                
def rotate(nodeid,endlocation,localToGlobal,caliberate):
    mapid=getMapIdByNodeId(nodeid)
    startlocation=caliberate[mapid]
    degree=localToGlobal[mapid]["rotate"]

    # Input values
    x1, y1 = startlocation[0], startlocation[1]  # Example initial values for x1, y1
    x2, y2 = endlocation[0], endlocation[1]  # Example values for x2, y2
    theta = math.radians(degree)  # Angle in radians (convert from degrees if needed)

    # Transformation matrix and calculations
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # Transformation equation: [x3, y3]
    x3 = cos_theta * (x2 - x1) + sin_theta * (y2 - y1) + x1
    y3 = -sin_theta * (x2 - x1) + cos_theta * (y2 - y1) + y1

    # Output result

    result=[x3,y3,0]

    return result




def ConvertManually(from_path,to_path,localToGLobal,sequence):
    data=loadJSONFile(from_path)
    entryPoints=getEntryPoints(data)
    print(entryPoints)

    #calculate start location based on go through what sequence of entry nodes
    #return a start location dict
    #caclulate "new position"
    #rotate

    caliberate={}
    for index,mapid in enumerate(sequence):
        prevMapid=sequence[index-1]
        print(prevMapid)
        #find
        for n in (entryPoints.keys()):
            if(getMapIdByNodeId(n)==mapid):
                #if last map
                #find all id with mapid 
                all=[]
                for e in entryPoints.keys():
                    if getMapIdByNodeId(e)==mapid:
                        all.append(e)
                for w in all:
                    if(getMapIdByNodeId(entryPoints[w])==prevMapid):
                        caliberate[mapid]=data[w]["local_location"]
                        
                
    #recalebrate and rotate

    nodelist=list(data.keys())
    locationlist=[]
    graph={}
    for index,nodeid in enumerate(nodelist):
        thisMapId=getMapIdByNodeId(nodeid)
        edges=[]
        for eId in data[nodeid]["edges"]:
            edges.append(nodelist.index(eId))

        
        location=data[nodeid]["local_location"]
        newLocation=rotate(nodeid,location,localToGLobal,caliberate)
        newLocation=[newLocation[0]+localToGLobal[thisMapId]["translate"][0]
            ,newLocation[1]+localToGLobal[thisMapId]["translate"][1]
            ,newLocation[2]+localToGLobal[thisMapId]["translate"][2]
        ]
        locationlist.append(newLocation)

    # connect entry points
    



        if(str(index) not in graph.keys()):
                graph[str(index)]={}
        graph[str(index)]["edges"]=edges

    outputdata={"node_locations":locationlist,"graph":graph}
    saveJSONAt(outputdata,to_path)
    


    
def getIdAndIndexByNodeId(nodeid):
    mapid,index=nodeid.split("_")
    return mapid,index

def getMapIdByNodeId(nodeid):            
            
           
    mapid,_=nodeid.split("_")
    return mapid


def convertToCuOptFormat(from_path,to_path):
    data=loadJSONFile(from_path)
    nodelist=list(data.keys())
    locationlist=[]
    graph={}
    localToGlobalLocation={}
    for index,nodeid in enumerate(nodelist):

        edges=[]
        for eId in data[nodeid]["edges"]:

            edges.append(nodelist.index(eId))

        if(str(index) not in graph.keys()):
            graph[str(index)]={}
        graph[str(index)]["edges"]=edges
        location=data[nodeid]["local_location"]
        locationlist.append(location)
    outputdata={"node_locations":locationlist,"graph":graph}
    saveJSONAt(outputdata,to_path)


def ConvertFormat(name):
    
    from_path="src/routing_engine/test_run/sample_data/"+name+".json"
    to_path=name+".json"

    localToGLobal={"000":{"translate":[11.60821,5.28206,0],"rotate":-90},
                           "001":{"translate":[0,0,0],"rotate":-180},
                           "002":{"translate":[13.04251,-5.13223,0],"rotate":-90},
                           }

    sequence=["001","000","002"]
    #ConvertAutomatically(from_path,to_path)
    ConvertManually(from_path,to_path,localToGLobal,sequence)
    #convertToCuOptFormat()
def ConfigureMap(name):
    from_path="src/routing_engine/test_run/sample_data/"+name+".json"
    to_path="src/routing_engine/test_run/sample_data/"+name+".json"
    data=loadJSONFile(from_path)
    graph=loadWaypointGraphData(data)

    offset=[]
    graph.getNodeById("000_000").localLocation

    

if __name__=="__main__":
    name=new
    ConvertFormat(name)
    #ConfigureMap(name)
