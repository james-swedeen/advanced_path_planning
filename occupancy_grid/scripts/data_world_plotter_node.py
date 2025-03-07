#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
import uuid
import csv
from visualization_msgs.msg import MarkerArray, Marker

class WorldPlotterNode(Node):
    """ Plots the world used for planning

        Publications:
            /world_viz: (visualization_msgs/msg/MarkerArray) Visualization message for each of the buildings

        Services:
            /regen_world: (Empty) Regenerates the world
    """

    def __init__(self) -> None:
        """ Initializes the subscription, publication, and service server
        """

        # Initialize the node
        super().__init__(node_name="world_plotter")

        # Create parameter for simulation time
        self._ts = -1.0
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value
        self.add_on_set_parameters_callback(self._param_callback)

        # Create parameter for data file
        self._data_file_name = ''
        self.declare_parameter(name="data_file", value=self._data_file_name, descriptor=ParameterDescriptor(description='The CSV file that hold the building data to plot'))
        self._data_file_name = self.get_parameter("data_file").value

        # Create parameter for map specifications
        self._max_north = 0.0
        self._max_east  = 0.0
        self._min_north = 0.0
        self._min_east  = 0.0
        self.declare_parameter(name="max_north", value=self._max_north, descriptor=ParameterDescriptor(description='The max north value for the building centers'))
        self.declare_parameter(name="max_east",  value=self._max_east,  descriptor=ParameterDescriptor(description='The max east value for the building centers'))
        self.declare_parameter(name="min_north", value=self._min_north, descriptor=ParameterDescriptor(description='The min north value for the building centers'))
        self.declare_parameter(name="min_east",  value=self._min_east,  descriptor=ParameterDescriptor(description='The min east value for the building centers'))
        self._max_north = self.get_parameter("max_north").value
        self._max_east  = self.get_parameter("max_east"). value
        self._min_north = self.get_parameter("min_north").value
        self._min_east  = self.get_parameter("min_east"). value
        assert self._max_north > self._min_north
        assert self._max_east  > self._min_east

        # Create message for publishing
        self._world_msg = self._genWorldMsg()

        # Create the publishers
        self._pub_world_viz = self.create_publisher(MarkerArray, "world_viz", 1)
        self._main_timer = self.create_timer(self._ts, self._main_loop)

    def _param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Default the success as true
        successful = True

        # Loop through any parameters and set them accordingly
        for param in params:
            match param.name:
                # Process the simulation period
                case "ts":
                    # The simulation period must be greater than zero
                    if param.value > 0.:
                        # Only update the simulation period if it is new
                        if self._ts != param.value:
                            self._ts = param.value # Update the node value

                            # Recreate the timer
                            self.destroy_timer(self._main_timer)
                            self._main_timer = self.create_timer(self._ts, self._main_loop)
                    else:
                        successful = False

        return SetParametersResult(successful=successful)

    def _genWorldMsg(self) -> MarkerArray:
        # Create the marker visualization message
        msg = MarkerArray()

        # Loop through all of the buildings and add them to the marker array
        counter = 0
        with open(self._data_file_name) as input_file:
            input_csv = csv.DictReader(input_file, delimiter=',')

            for building_it in input_csv:
                bldg = Marker()
                # Building position (recall that the position is in the center of each dimension, thus way each dimension has a + val/2)
                bldg.pose.position.x = float(building_it['north'])
                bldg.pose.position.y = float(building_it['east'])
                bldg.pose.position.z = -float(building_it['height'])/2
                # Check if the building is in bounds
                if ((bldg.pose.position.x > self._max_north) or
                    (bldg.pose.position.x < self._min_north) or
                    (bldg.pose.position.y > self._max_east)  or
                    (bldg.pose.position.y < self._min_east)):
                    continue
                # Create the generic parameters
                bldg.header.frame_id = "ned"
                bldg.header.stamp = self.get_clock().now().to_msg()
                bldg.ns = "world"
                bldg.id = counter
                bldg.type = Marker.CYLINDER
                bldg.action = Marker.ADD
                # Building size
                bldg.scale.x = float(building_it['n_width'])
                bldg.scale.y = float(building_it['e_width'])
                bldg.scale.z = float(building_it['height'])
                # Building color
                bldg.color.a = 1.0
                bldg.color.r = 1.0
                bldg.color.g = 0.0
                bldg.color.b = 0.0

                bldg.frame_locked = True

                msg.markers.append(bldg)
                counter += 1

        # Add the region boundary
        bound = Marker()
        # Create the generic parameters
        bound.header.frame_id = "ned"
        bound.header.stamp = self.get_clock().now().to_msg()
        bound.ns = "boundary"
        bound.type = Marker.LINE_STRIP
        bound.action = Marker.ADD
        bound.id = counter
        # Color
        bound.color.a = 1.0
        bound.color.r = 0.0
        bound.color.g = 1.0
        bound.color.b = 0.0
        bound.frame_locked = True
        # Line width
        bound.scale.x = 10.0
        point0 = Point()                # Origin
        point0.x = self._min_north
        point0.y = self._min_east
        point0.z = 0.0
        bound.points.append(point0)
        point1 = Point()                # East corner
        point1.x = self._max_north
        point1.y = self._min_east
        point1.z = 0.0
        bound.points.append(point1)
        point2 = Point()                # North-East corner
        point2.x = self._max_north
        point2.y = self._max_east
        point2.z = 0.0
        bound.points.append(point2)
        point3 = Point()                # North corner
        point3.x = self._min_north
        point3.y = self._max_east
        point3.z = 0.0
        bound.points.append(point3)
        bound.points.append(point0)     # Complete loop

        msg.markers.append(bound)

        return msg


    def _main_loop(self) -> None:
        """ Publishes the latest world message
        """
        # Publish the messages
        self._pub_world_viz.publish(self._world_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    world_plotter_node = WorldPlotterNode()
    rclpy.spin(world_plotter_node)

    # Shutdown the node
    world_plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
