#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pluginlib import PluginlibHelper

def main():
    rclpy.init()
    node = Node('plugin_test')
    
    try:
        plugin_helper = PluginlibHelper(node=node)
        plugins = plugin_helper.get_plugins_by_type('hardware_interface::SystemInterface')
        print("\nFound plugins:")
        for plugin in plugins:
            print(f"- {plugin}")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 