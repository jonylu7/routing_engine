import unittest
import elkai
import numpy as np
from routing_agent.Node import Node
import routing_agent.FindPath
from routing_agent.RoutingEngine import RoutingEngine
from routing_agent.ConvertDataFormat import loadJSONFile
from routing_agent.Task import Task,loadTasksData
from routing_agent.Vehicle import Vehicle,loadVehiclesData
from routing_agent.WaypointGraph import *

class TestRoutingEngine(unittest.TestCase):
    def test_route_to_the_end(self):
        data = loadJSONFile("../../test_run/sample_data/waypointgraph.json")
        graph = loadWaypointGraphData(data)
        # print(convertWaypointGraphToJSON(graph))
        vehdata = loadJSONFile("../../test_run/sample_data/vehicle_data.json")
        vehicles = loadVehiclesData(graph, vehdata)
        taskdata = loadJSONFile("../../test_run/sample_data/task_data.json")
        tasks = loadTasksData(graph, taskdata)
        re = RoutingEngine(graph, tasks, vehicles)
        print(re.update("000_000"))
        print(re.response())
        print(re.update("000_003"))
        print(re.response())
        print(re.update("001_006"))
        print(re.update("002_001"))





if __name__ == '__main__':
    unittest.main()