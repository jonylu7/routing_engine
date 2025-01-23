# Copyright (c) 2021-2023, NVIDIA CORPORATION.

from pxr import UsdGeom, UsdLux
import omni.ext
from omni.kit.menu.utils import (
    add_menu_items,
    remove_menu_items,
    MenuItemDescription,
)

import omni.ui as ui
from omni.isaac.core.utils.nucleus import (
    get_assets_root_path,
    get_nvidia_asset_root_path,
)
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.ui.ui_utils import (
    setup_ui_headers,
    get_style,
    btn_builder,
    str_builder,
)

import gc
import weakref

import json
from omni.isaac.core.utils.extensions import enable_extension


from omni.cuopt.service.waypoint_graph_model import (
    WaypointGraphModel,
    load_waypoint_graph_from_file,
)
from omni.cuopt.service.transport_orders import TransportOrders
from omni.cuopt.service.transport_vehicles import TransportVehicles
from omni.cuopt.service.cuopt_data_proc import preprocess_cuopt_data
from omni.cuopt.service.cuopt_microservice_manager import cuOptRunner
from omni.cuopt.service.common import (
    show_vehicle_routes,
    test_connection_microservice,
    test_connection_managed_service
)

from omni.cuopt.visualization.common import check_build_base_path
from omni.cuopt.visualization.generate_warehouse_building import (
    generate_building_structure,
)
from omni.cuopt.visualization.generate_warehouse_assets import (
    generate_shelves_assets,
    generate_conveyor_assets,
)
from omni.cuopt.visualization.generate_waypoint_graph import (
    visualize_waypoint_graph,
    update_weights,
)
from omni.cuopt.visualization.generate_orders import visualize_order_locations
from omni.cuopt.visualization.generate_semantics import generate_semantic_zones



EXTENSION_NAME = "Intra-warehouse Transport Demo"

from  routing_agent.LoadWaypointGraphClient import LoadWaypointGraphClient
from routing_agent.RoutingServer import main as StartupRoutingServer


class louieRoutingServiceExtension():
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        
        
        self._ext_id = ext_id

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._extension_data_path = f"{self._extension_path}/omni/cuopt/examples/warehouse_transport_demo/extension_data/"

        self._window = None
        self._usd_context = omni.usd.get_context()

        self._user_load_order_location_x="undefined"
        self._user_load_order_location_y="undefined"
        self._user_input_order_location=[]

        self._semantic = {}

        base_isaac_path = get_assets_root_path()
        base_nvidia_path = get_nvidia_asset_root_path()

        self._isaac_asset_path = base_isaac_path + "/Isaac/"
        self._isaac_nvidia_asset_path = base_isaac_path + "/NVIDIA/"
        self._nvidia_digital_twin_path = (
            base_nvidia_path + "/Assets/DigitalTwin/Assets/Warehouse/"
        )

        self.waypoint_graph_node_path = (
            "/World/Warehouse/Transportation/WaypointGraph/Nodes"
        )
        self.waypoint_graph_edge_path = (
            "/World/Warehouse/Transportation/WaypointGraph/Edges"
        )

        self.warehouse_building_config = "warehouse_building_data.json"
        self.warehouse_shelves_config = "warehouse_shelves_data.json"
        self.warehouse_conveyors_config = "warehouse_conveyors_data.json"
        new="waypointgraph_new_1.json"
        indoor="waypointgraph_indoor.json"
        outdoor="waypointgraph_outdoor.json"
        og="waypointgraph.json"
        unfin="waypointgraph_new_1_unshifted.json"
        #self.waypoint_graph_config = "converted_formats/"+new
        self.waypoint_graph_config = "waypoint_graph.json"
        self.semantic_config = "semantics_data.json"
        self.orders_config = "orders_data.json"
        self.vehicles_config = "vehicle_data.json"

        self._waypoint_graph_model = WaypointGraphModel()
        self._orders_obj = TransportOrders()
        self._vehicles_obj = TransportVehicles()    
        self._semantics = []

        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_NAME,
                onclick_fn=lambda a=weakref.proxy(self): a._menu_callback(),
            )
        ]

        add_menu_items(self._menu_items, "RoutingEngine")

        self._build_ui()

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        if self._window.visible:
            self._sub_stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
                self._on_stage_event
            )
        else:
            self._sub_stage_event = None

    def _build_ui(self):

        if not self._window:
            self._window = ui.Window(
                title=EXTENSION_NAME,
                width=0,
                height=0,
                visible=False,
                dockPreference=ui.DockPreference.LEFT_BOTTOM,
            )
            self._window.set_visibility_changed_fn(self._on_window)

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                title = "cuOpt Extension Code and Docs"
                doc_link = "https://docs.nvidia.com/cuopt/"

                overview = "This example demonstrates use of the NVIDIA cuOpt microservice "
                overview += "to perform routing optimization in an intra-warehouse transport context."
                overview += "\n\nPress the 'Open Source Code' button to view the source code."

                setup_ui_headers(
                    self._ext_id, __file__, title, doc_link, overview
                )

                #---start modify by Lu---

                user_input_waypoint = ui.CollapsableFrame(
                    title="User Input Orders",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with user_input_waypoint:

                    ui_data_style = {
                            "font_size": 14,
                            "color": 0x88888888,
                            "alignment": ui.Alignment.LEFT,
                        }

                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        kwargs = {
                            "label": "Input Waypoint Location, X",
                            "type": "stringfield",
                            "default_val": "x",
                            "tooltip": "SAK for cuOpt managed service",
                            "on_clicked_fn": None,
                            "use_folder_picker": False,
                            "read_only": False,
                        }
                        self._user_load_order_location_x= str_builder(**kwargs)

                        kwargs = {
                            "label": "Input Waypoint Location, Y",
                            "type": "stringfield",
                            "default_val": "y",
                            "tooltip": "Function name for cuOpt managed service",
                            "on_clicked_fn": None,
                            "use_folder_picker": False,
                            "read_only": False,
                        }
                        self._user_load_order_location_y = str_builder(**kwargs)

                        kwargs = {
                            "label": "Load Input Location",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Load just inputed location",
                            "on_clicked_fn": self._load_order_location,
                        }
                        btn_builder(**kwargs)
                        self._user_load_order_ui_data=ui.Label(
                            "No Orders Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Generate OrderJSON",
                            "type": "button",
                            "text": "Generate",
                            "tooltip": "generate JSON file",
                            "on_clicked_fn": self._generate_order_json,
                        }
                        btn_builder(**kwargs)
                        

                #---end modify by Lu---
            
                # Setting up the UI to connect to cuOpt Managed Service
                

                # Setting up the UI setup the optimization problem
                setup_frame = ui.CollapsableFrame(
                    title="Optimization Problem Setup",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with setup_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        ui_data_style = {
                            "font_size": 14,
                            "color": 0x88888888,
                            "alignment": ui.Alignment.LEFT,
                        }

                        kwargs = {
                            "label": "Startup Routing Server ",
                            "type": "button",
                            "text": "Startup",
                            "tooltip": "startup run routing engine server",
                            "on_clicked_fn": self._startup_routing_engine_server,
                        }
                        btn_builder(**kwargs)
                        self._server_ui_data = ui.Label(
                            "Not Yet Started Server",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Load Sample Warehouse ",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads an example warehouse environment",
                            "on_clicked_fn": self._build_warehouse_environment,
                        }
                        btn_builder(**kwargs)
                        self._warehouse_ui_data = ui.Label(
                            "No Warehouse Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Startup server ",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Startup ",
                            "on_clicked_fn": self._load_waypoint_graph,
                        }
                        btn_builder(**kwargs)
                        self._startup_server = ui.Label(
                            "No Waypoint Graph network Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )
                        

                        kwargs = {
                            "label": "Load Waypoint Graph ",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads a waypoint graph for the sample environment",
                            "on_clicked_fn": self._load_waypoint_graph,
                        }
                        btn_builder(**kwargs)
                        self._network_ui_data = ui.Label(
                            "No Waypoint Graph network Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Load Orders ",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads sample orders",
                            "on_clicked_fn": self._load_orders,
                        }
                        btn_builder(**kwargs)
                        self._orders_ui_data = ui.Label(
                            "No Orders Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Load Vehicles ",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads sample vehicle data",
                            "on_clicked_fn": self._load_vehicles,
                        }
                        btn_builder(**kwargs)
                        self._vehicle_ui_data = ui.Label(
                            "No Vehicles Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        kwargs = {
                            "label": "Generate Semantic Zone ",
                            "type": "button",
                            "text": "Generate",
                            "tooltip": "Generates a sample semantics zone",
                            "on_clicked_fn": self._load_semantic_zone,
                        }
                        btn_builder(**kwargs)
                        self._semantic_ui_data = ui.Label(
                            "No Semantic Zones Loaded",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

                        with ui.VStack(style=get_style(), spacing=5, height=0):

                            kwargs = {
                                "label": "length",
                                "type": "integer",
                                "default_val": 2.0,
                                "tooltip": "Length for semantic",
                                "on_clicked_fn": None,
                                "use_folder_picker": False,
                                "read_only": False,
                            }
                            self._semantic["length"] = str_builder(**kwargs)

                            kwargs = {
                                "label": "width",
                                "type": "integer",
                                "default_val": 2.0,
                                "tooltip": "Width for semantic",
                                "on_clicked_fn": None,
                                "use_folder_picker": False,
                                "read_only": False,
                            }
                            self._semantic["width"] = str_builder(**kwargs)

                # Setting up the UI setup the optimization problem
                run_frame = ui.CollapsableFrame(
                    title="Update/Run cuOpt",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with run_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        ui_data_style = {
                            "font_size": 14,
                            "color": 0xBBBBBBBB,
                            "alignment": ui.Alignment.LEFT,
                        }

                        kwargs = {
                            "label": "Update Weights ",
                            "type": "button",
                            "text": "Update",
                            "tooltip": "Update the waypoint graph weights",
                            "on_clicked_fn": self._update_weights,
                        }
                        btn_builder(**kwargs)

                        kwargs = {
                            "label": "Run cuOpt ",
                            "type": "button",
                            "text": "Solve",
                            "tooltip": "Run the cuOpt solver based on current data",
                            "on_clicked_fn": self._run_cuopt,
                        }
                        btn_builder(**kwargs)
                        self._routes_ui_message = ui.Label(
                            "Run cuOpt for solution",
                            width=350,
                            word_wrap=True,
                            style=ui_data_style,
                        )

    def _on_stage_event(self, event):
        """
        Function for monitoring stage events
        """
        if event.type == 2:
            self._semantics = []

        # print(f"stage event type int: {event.type}{event.payload}")


    def _startup_routing_engine_server(self):
        try:
            StartupRoutingServer()
        finally:
             self._server_ui_data.text="Starting Server Successfully"
            
    def _build_warehouse_environment(self):
        print("building environment")
        
        building_json_path = (
            f"{self._extension_data_path}{self.warehouse_building_config}"
        )

        self._stage = self._usd_context.get_stage()

        building_prim_path = "/World/Warehouse/Building"
        check_build_base_path(
            self._stage, building_prim_path, final_xform=True
        )

        generate_building_structure(
            self._stage,
            building_prim_path,
            building_json_path,
            self._isaac_asset_path,
        )
        
        
        # Add outdoor lighting via hdr
        sky_light_stage_path = "/World/ExteriorHDR"

        hdr_path = (
            self._isaac_nvidia_asset_path
            + "Assets/Skies/Clear/noon_grass_4k.hdr"
        )

        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=sky_light_stage_path,
            prim_type="DomeLight",
            select_new_prim=False,
            attributes={
                UsdLux.Tokens.inputsIntensity: 1000,
                UsdLux.Tokens.inputsSpecular: 1,
                UsdLux.Tokens.inputsTextureFile: hdr_path,
                UsdLux.Tokens.inputsTextureFormat: UsdLux.Tokens.latlong,
                UsdGeom.Tokens.visibility: "inherited",
            },
            create_default_xform=True,
        )
        self._warehouse_ui_data.text = f"Warehouse loaded"

        set_camera_view(
            eye=[2.0, 7.0, 8.0],
            target=[26.0, 60.0, 0.0],
            camera_prim_path="/OmniverseKit_Persp",
        )
        


    def _test_load_waypoint_graph_through_cleint(self):
        #rclpy.init()
        loadClient = LoadWaypointGraphClient()
        loadClient.send_request([self.waypoint_graph_config])

        rclpy.spin_once(loadClient)
        if loadClient.future.done():
            try:
                response = loadClient.future.result()
                print(response)
            except Exception as e:
               print('Service call failed %r' % (e,))
            else:
                return response.can_load

        loadClient.destroy_node()
        rclpy.shutdown()

    def _load_waypoint_graph(self):
        print("loading waypoint graph")
        
        self._test_load_waypoint_graph_through_cleint()


        self._stage = self._usd_context.get_stage()
        waypoint_graph_data_path = (
            f"{self._extension_data_path}{self.waypoint_graph_config}"
        )
        self._waypoint_graph_model = load_waypoint_graph_from_file(
            self._stage, waypoint_graph_data_path
        )
        visualize_waypoint_graph(
            self._stage,
            self._waypoint_graph_model,
            self.waypoint_graph_node_path,
            self.waypoint_graph_edge_path,
        )
        self._network_ui_data.text = f"Waypoint Graph Network Loaded: {len(self._waypoint_graph_model.nodes)} nodes, {len(self._waypoint_graph_model.edges)} edges"

    def _load_orders(self):
        print("Loading Orders")
        orders_path = f"{self._extension_data_path}{self.orders_config}"
        self._orders_obj.load_sample(orders_path)
        visualize_order_locations(
            self._stage, self._waypoint_graph_model, self._orders_obj
        )
        self._orders_ui_data.text = f"Orders Loaded: {len(self._orders_obj.graph_locations)} tasks at nodes {self._orders_obj.graph_locations}"

    def _load_vehicles(self):
        print("Loading Vehicles")
        vehicle_data_path = (
            f"{self._extension_data_path}{self.vehicles_config}"
        )
        self._vehicles_obj.load_sample(vehicle_data_path)
        start_locs = [locs[0] for locs in self._vehicles_obj.graph_locations]
        self._vehicle_ui_data.text = f"Vehicles Loaded: {len(self._vehicles_obj.graph_locations)} vehicles at nodes {start_locs}"

    def _load_semantic_zone(self):
        length = self._semantic["length"].get_value_as_float()
        width = self._semantic["width"].get_value_as_float()
        semantic_prim_path = "/World/Warehouse/Semantics"
        check_build_base_path(
            self._stage, semantic_prim_path, final_xform=True
        )

        self._semantics = generate_semantic_zones(
            self._stage, semantic_prim_path, self._semantics, length, width
        )
        self._semantic_ui_data.text = f"Semantic Zones loaded"

    # Update the network edge weights based on semantics
    def _update_weights(self):
        print("updating weights")
        self._stage = self._usd_context.get_stage()
        update_weights(
            self._stage, self._waypoint_graph_model, self._semantics
        )


    def _load_order_location(self):
        if(self._user_load_order_location_x.get_value_as_string()=="x" or self._user_load_order_location_y.get_value_as_string()=="y"):
            raise KeyError("fuck you, you need to input both location x AND y, fucking moron")
            return
        orderX=float(self._user_load_order_location_x.get_value_as_string())
        orderY=float(self._user_load_order_location_y.get_value_as_string())
        
        orderLocation=[orderX,orderY,0]
        if(orderLocation in self._user_input_order_location):
            raise KeyError("fuck you, you already input this location x y, fucking moron")
            return
        self._user_input_order_location.append(orderLocation)
        self._user_load_order_ui_data.text=f"Total orders: {len( self._user_input_order_location)}, Orders Location:{self._user_input_order_location}, Orders Id:"

    def _generate_order_json(self):
        if(len(self._user_input_order_location)==0):
            raise KeyError("fuck you, you need to input order data, fucking moron")
            return
        data={"task_locations":self._user_input_order_location,"demand":[1]*len(self._user_input_order_location)}
        
        print(data)
        json_object = json.dumps(data, indent=2)
        
        exportOrderLocation=self._extension_data_path+"orders_data.json"
        # Writing to sample.json
        with open(exportOrderLocation, "w") as outfile:
            outfile.write(json_object)
       



    def _run_cuopt(self):
        print("Running cuOpt")

        self._stage = self._usd_context.get_stage()

        # Solver Settings
        solver_config = {
            "time_limit": 0.01,
        }

        # Preprocess network, fleet and task data
        waypoint_graph_data, fleet_data, task_data = preprocess_cuopt_data(
            self._waypoint_graph_model, self._orders_obj, self._vehicles_obj
        )

        # Initialize server data and call for solve
        environment_data = {
            "cost_waypoint_graph_data": waypoint_graph_data,
            "fleet_data": fleet_data,
            "task_data": task_data,
            "solver_config": solver_config,
        }
       #---start modify by Lu---
        #Client.runORAgent(self._extension_data_path+"waypoint_graph.json",self._extension_data_path+"orders_data.json",self._extension_data_path+"vehicle_data.json",self._extension_data_path+"ORAgent_response.json")
        with open(self._extension_data_path+"ORAgent_response.json","r")as f:
            data=json.load(f)
        routes=data["response"]["solver_response"]
        #---end modify by Lu---
        #if self.client is None:
            
            
           

        #else:
        #    res = self.client.get_optimized_routes(environment_data)
        #    routes = res["response"]["solver_response"]
        
        

        # Display the routes on UI
        self._routes_ui_message.text = show_vehicle_routes(routes)
        #print(routes)
        # Visualize the optimized routes
        self._waypoint_graph_model.visualization.display_routes(
            self._stage,
            self._waypoint_graph_model,
            self.waypoint_graph_edge_path,
            routes,
        )

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "cuOpt")
        self._window = None
        gc.collect()
