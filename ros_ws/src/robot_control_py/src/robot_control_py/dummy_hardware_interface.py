#!/usr/bin/env python3

from hardware_interface.system_interface import SystemInterface
from hardware_interface.hardware_interface_return_values import HardwareInterfaceReturnValues

class DummyHardwareInterface(SystemInterface):
    """
    Dummy hardware interface for testing plugin loading.
    """

    def __init__(self):
        super().__init__()
        print("DummyHardwareInterface::__init__")

    def on_init(self, hardware_info):
        print("DummyHardwareInterface::on_init")
        return HardwareInterfaceReturnValues.SUCCESS

    def export_state_interfaces(self):
        print("DummyHardwareInterface::export_state_interfaces")
        return []

    def export_command_interfaces(self):
        print("DummyHardwareInterface::export_command_interfaces")
        return []

    def read(self, time, period):
        print("DummyHardwareInterface::read")
        return HardwareInterfaceReturnValues.SUCCESS

    def write(self, time, period):
        print("DummyHardwareInterface::write")
        return HardwareInterfaceReturnValues.SUCCESS

def export_plugin(plugin_name="robot_control_py/DummyHardwareInterface"):
    return DummyHardwareInterface