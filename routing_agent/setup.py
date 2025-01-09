from setuptools import find_packages, setup

package_name = 'routing_agent'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/InitAndRun.launch.py']),
         ('share/' + package_name, ['launch/RunServer.launch.py']),
        #('lib/' + package_name, [package_name+'/__init__.py']),
        ('lib/' + package_name, [package_name+'/Task.py']),
        ('lib/' + package_name, [package_name+'/Vehicle.py']),
         ('lib/' + package_name, [package_name+'/Vector.py']),
          
        ('lib/' + package_name, [package_name+'/Node.py']),
        ('lib/' + package_name, [package_name+'/WaypointGraph.py']),
        ('lib/' + package_name, [package_name+'/RoutingEngine.py']),
        ('lib/' + package_name, [package_name+'/RoutingClient.py']),
         ('lib/' + package_name, [package_name+'/LoadWaypointGraphClient.py']),
          ('lib/' + package_name, [package_name+'/MergeWaypointGraphClient.py']),
          ('lib/' + package_name, [package_name+'/NavClient.py']),
         ('lib/' + package_name, [package_name+'/RoutingServer.py']),
         ('lib/' + package_name, [package_name+'/LoadToolkit.py']),

#('lib/' + package_name+"/test", ['test/test_routingengine_with_indoor.py']),
('lib/'+ package_name, ['test/test_routingengine_with_new_1.py']),
#('lib/' + package_name+"/test", ['test/test_routingengine_with_indoor.py']),

('lib/' + package_name+"/utils", [package_name+'/utils/__init__.py']),
('lib/' + package_name+"/utils", [package_name+'/utils/PreprocessToolkit.py']),
 ('lib/' + package_name+"/utils", [package_name+'/utils/FindPath.py']),
 ('lib/' + package_name+"/utils", [package_name+'/utils/OR.py']),
  ('lib/' + package_name+"/utils", [package_name+'/utils/ConvertDataFormat.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='csl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = routing_agent.RoutingServer:main',
            'routingClient = routing_agent.RoutingClient:main',
            'loadGraph = routing_agent.LoadWaypointGraphClient:main',
            'mergeGraph = routing_agent.MergeWaypointGraphClient:main',
            'nav = routing_agent.NavClient:main'
        ],
    },
)
