
import unittest
import time
import launch
import launch_testing.actions
import rclpy
import routing_agent.NavClient
from turtlesim.msg import Pose
from launch.event_handlers import OnProcessExit,OnExecutionComplete,OnProcessStart
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument,ExecuteProcess
import routing_agent

def generate_test_description():

    runServer=ExecuteProcess(cmd=[['ros2 run routing_agent server']],shell=True)

    loadGraphArg=ExecuteProcess(cmd=[['ros2 run routing_agent loadGraph src/routing_engine/test_run/sample_data/waypointgraph.json']],shell=True)
    loadTaskAndVehicleArg=ExecuteProcess(cmd=[['ros2 run routing_agent routingClient src/routing_engine/test_run/sample_data/task_data.json src/routing_engine/test_run/sample_data/vehicle_data.json']],shell=True)

    # Define dependencies using event handlers
    process_2_after_1 = RegisterEventHandler(
        OnProcessStart(
            target_action=runServer,
            on_start=[loadGraphArg]
        )
    )

    process_3_after_2 = RegisterEventHandler(
        OnExecutionComplete(
            target_action=loadGraphArg,
            on_completion=[loadTaskAndVehicleArg]
        )
    )


    return LaunchDescription([
        runServer,
        process_2_after_1,
        process_3_after_2,
launch_testing.actions.ReadyToTest()])


# Active tests
class TestRoutingEngineWithNew1Map(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = routing_agent.NavClient()

    def tearDown(self):
        self.node.destroy_node()


    def test_publishes_pose(self, proc_output):

        """Check whether pose messages published"""
        self.node.send_request(["T","000_001"])

        try:
            end_time = time.time() + 2
            while time.time() < end_time:
                response = self.node.future.result()
                self.node.get_logger().info(
                        'PathToNextTask: '+response.path_to_next_task+"\n")  # CHANGE
            assert response.path_to_next_task =="[]"
           
        finally:
            self.tearDown()
            self.tearDownClass()




class TestSimple(unittest.TestCase):
    def test_Simple(self):
        assert True==1




    

