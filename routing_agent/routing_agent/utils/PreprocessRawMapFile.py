from Vector import Vector3
import math
from routing_agent.routing_agent.utils.PreprocessToolkit import *
from ConvertDataFormat import *
from pathlib import Path

def generate_intermediate_nodes_index(startVector, endVector, spacing):
    """
    Generate intermediate nodes between two points with a given spacing.

    Args:
    startVector (tuple): Coordinates of the starting node (startVector.x, startVector.y).
    endVector (tuple): Coordinates of the ending node (endVector.x, endVector.y).
    spacing (float): Desired spacing between consecutive nodes.

    Returns:
    list: A list of tuples representing the intermediate nodes, including the start and end nodes.
    """

    # Calculate the distance between the two nodes
    distance = math.sqrt((endVector.x - startVector.x) ** 2 + (endVector.y - startVector.y) ** 2)

    # Determine the number of segments
    if distance == 0 or spacing <= 0:
        raise ValueError("Spacing must be greater than 0 and nodes must not be the same.")

    num_segments = int(distance // spacing)

    # Generate intermediate points
    intermediate_nodes = [
        Vector3(
            startVector.x + i * spacing * (endVector.x - startVector.x) / distance,
            startVector.y + i * spacing * (endVector.y - startVector.y) / distance
        )
        for i in range(num_segments + 1)
    ]
    # Add the endpoint if it's not exactly at spacing distance
    if intermediate_nodes[-1] != endVector:
        intermediate_nodes.append(endVector)

    return intermediate_nodes

def preprocess():

    spacing=5
    filepath = Path("src/routing_agent/routing_agent/preprocess_maps/waypoint_graph_raw.yaml")

    yamlFile=loadYAMLFile(filepath)
    maps = yamlFile["entry_points"]
    graph=WaypointGraph()
    for map in maps:
        mapId = map["map_id"]
        id = []
        index=0
        nodeList = []
        lastEnd=Vector3(-1,-1,-1)
        for i in range(len(map["points"]) - 1):
            start = Vector3(map["points"][i]["coordinates"][0], map["points"][i]["coordinates"][1])
            end = Vector3(map["points"][i + 1]["coordinates"][0], map["points"][i + 1]["coordinates"][1])
            nodes = generate_intermediate_nodes_index(start, end, spacing)
            if(lastEnd==start):
                nodes=nodes[1::]

            for n in nodes:
                nodeid=generateNodeId(mapId,index)
                node=Node(id=nodeid,locallocation=n,mapid=mapId,isentrypoint=False)
                nodeList.append(node)
                index+=1

            lastEnd=end

        for i in range(len(nodeList)-1):
            connectTwoNodes(nodeList[i],nodeList[i+1])
            print(nodeList[i].id)
            print(nodeList[i].edges)
            graph.addNode(nodeList[i])
        graph.addNode(nodeList[len(nodeList)-1])

    print(graph.convertToJSON())

    graph.setEntryPoints("000_005","001_000")
    graph.setEntryPoints("001_008", "003_000")
    graph.setEntryPoints("001_015", "002_000")
    graph.setEntryPoints("003_019", "004_000")
    graph.setEntryPoints("004_012", "003_019")

    print(graph.convertToJSON())
    saveJSONAt(graph.convertToJSON(), "test.json")
    return graph

