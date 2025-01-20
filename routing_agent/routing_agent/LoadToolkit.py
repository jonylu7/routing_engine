from routing_agent.WaypointGraph import WaypointGraph
from routing_agent.Vehicle import Vehicle
from routing_agent.utils.PreprocessToolkit import *

def loadVehiclesData(waypointGraph:WaypointGraph,vehicledata):
    vehicleSize=len(vehicledata["vehicle_locations"])
    fleet=[]
    for i in range(vehicleSize):
        node=waypointGraph.getNodeById(vehicledata["vehicle_locations"][i])
        capacity=vehicledata["vehicle_locations"][i]
        fleet.append(Vehicle(node,capacity))
    return fleet

def loadTasksData(waypointGraph:WaypointGraph,taskdata):
    taskSize=len(taskdata["task_sequence"])
    tasks=[]
    for i in range(taskSize):
        try:
            node=waypointGraph.getNodeById(taskdata["task_sequence"][i])
        except:
            raise KeyError("Failed to load task data, can't match task node with waypointgraph")
        demand=taskdata["task_sequence"][i]
        tasks.append(Task(node,demand))
    return tasks


def loadWaypointGraphData(waypointgraphdata:dict)->WaypointGraph:
    idlist=[]
    nodeList=[]
    for nodeid,value in waypointgraphdata.items():
        idlist.append(nodeid)
        nodeList.append(convertJSONnodeToNode(nodeid,value))

    return WaypointGraph(idlist,nodeList)
