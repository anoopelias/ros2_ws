#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
import launch
from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.events.process import ShutdownProcess
from launch.events.matchers import matches_action
from launch.event_handlers import OnProcessStart, OnProcessExit
from functools import partial
import threading
import asyncio


class DynamicLauncher(Node):
    def __init__(self):
        super().__init__('dynamic_launcher')

        # Keep track of running nodes and their launch actions
        self.running_nodes = {}

        # Services for starting and stopping nodes
        self.start_service = self.create_service(
            SetBool,
            'start_node',
            self.start_node_callback
        )

        self.stop_service = self.create_service(
            SetBool,
            'stop_node',
            self.stop_node_callback
        )

        # Publisher to announce node status
        self.status_publisher = self.create_publisher(String, 'node_status', 10)

        self.get_logger().info('Dynamic Launcher ready - use start_node and stop_node services')
        self.get_logger().info('Example: ros2 service call /start_node std_srvs/srv/SetBool "{data: true}"')

    def start_node_callback(self, request, response):
        """Service callback to start a node dynamically"""
        try:
            if request.data:
                # For demonstration, start the number_publisher node
                node_name = 'dynamic_number_publisher'

                if node_name in self.running_nodes:
                    response.success = False
                    response.message = f'Node {node_name} is already running'
                    return response

                # Create launch node action
                node_action = LaunchNode(
                    package='my_py_pkg',
                    executable='number_publisher',
                    name=node_name,
                    output='screen'
                )

                # Create launch description for the node
                launch_description = LaunchDescription([node_action])

                # Add event handlers for process start and exit
                start_handler = launch.actions.RegisterEventHandler(OnProcessStart(
                    target_action=node_action,
                    on_start=partial(self.start_callback, node_name)))

                exit_handler = launch.actions.RegisterEventHandler(OnProcessExit(
                    target_action=node_action,
                    on_exit=partial(self.stop_callback, node_name)))

                launch_description.add_action(start_handler)
                launch_description.add_action(exit_handler)

                # Add to launch service
                self.launch_service.include_launch_description(launch_description)

                # Store the node action for later shutdown
                self.running_nodes[node_name] = node_action

                # Publish status
                status_msg = String()
                status_msg.data = f'Started node: {node_name}'
                self.status_publisher.publish(status_msg)

                response.success = True
                response.message = f'Successfully started {node_name}'
                self.get_logger().info(f'Started node: {node_name}')

            else:
                response.success = False
                response.message = 'Set data to true to start a node'

        except Exception as e:
            response.success = False
            response.message = f'Failed to start node: {str(e)}'
            self.get_logger().error(f'Error starting node: {e}')

        return response

    def stop_node_callback(self, request, response):
        """Service callback to stop a node dynamically"""
        try:
            if request.data:
                node_name = 'dynamic_number_publisher'

                if node_name not in self.running_nodes:
                    response.success = False
                    response.message = f'Node {node_name} is not running'
                    return response

                # Get the node action
                node_action = self.running_nodes[node_name]

                # Create shutdown event
                shutdown_event = ShutdownProcess(process_matcher=matches_action(node_action))

                # Emit shutdown event
                self.launch_service.emit_event(shutdown_event)

                # Remove from tracking
                del self.running_nodes[node_name]

                # Publish status
                status_msg = String()
                status_msg.data = f'Stopped node: {node_name}'
                self.status_publisher.publish(status_msg)

                response.success = True
                response.message = f'Successfully stopped {node_name}'
                self.get_logger().info(f'Stopped node: {node_name}')

            else:
                response.success = False
                response.message = 'Set data to true to stop a node'

        except Exception as e:
            response.success = False
            response.message = f'Failed to stop node: {str(e)}'
            self.get_logger().error(f'Error stopping node: {e}')

        return response

    def start_callback(self, node_name, event, context):
        """Callback when process starts"""
        self.get_logger().info(f'Node process started: {node_name}')
        status_msg = String()
        status_msg.data = f'Process started for node: {node_name}'
        self.status_publisher.publish(status_msg)

    def stop_callback(self, node_name, event, context):
        """Callback when process exits"""
        self.get_logger().info(f'Node process exited: {node_name}')
        status_msg = String()
        status_msg.data = f'Process exited for node: {node_name}'
        self.status_publisher.publish(status_msg)

    def set_launch_service(self, launch_service):
        """Set the launch service from main thread"""
        self.launch_service = launch_service

    def destroy_node(self):
        """Clean shutdown"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create our ROS node
        launcher = DynamicLauncher()

        # Create launch service on main thread
        launch_service = LaunchService()
        launch_service.include_launch_description(LaunchDescription())
        launcher.set_launch_service(launch_service)

        # Spin ROS node in separate thread
        def spin_ros():
            rclpy.spin(launcher)

        ros_thread = threading.Thread(target=spin_ros)
        ros_thread.daemon = True
        ros_thread.start()

        # Run launch service on main thread
        launch_service.run(shutdown_when_idle=False)

    except KeyboardInterrupt:
        pass
    finally:
        if 'launcher' in locals():
            launcher.destroy_node()
        if 'launch_service' in locals():
            launch_service.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()