#!/usr/bin/env python3
"""Dump all CFG_* parameters from a running ublox_dgnss node to a YAML param file.

Usage:
  1. Start the driver: ros2 launch ublox_dgnss ublox_x20d_test.launch.py
  2. Wait a few seconds for config to load from device
  3. Run: python3 dump_x20d_config.py [node_name] [output_file]

  Defaults: node=/gnss/ublox_dgnss, output=x20d_config.yaml
"""
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters


def main():
    node_name = sys.argv[1] if len(sys.argv) > 1 else "/gnss/ublox_dgnss"
    output_file = sys.argv[2] if len(sys.argv) > 2 else "x20d_config.yaml"

    rclpy.init()
    node = Node("config_dumper")

    list_client = node.create_client(ListParameters, f"{node_name}/list_parameters")
    get_client = node.create_client(GetParameters, f"{node_name}/get_parameters")

    if not list_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Node {node_name} not available")
        rclpy.shutdown()
        return

    # List all parameters
    list_req = ListParameters.Request()
    future = list_client.call_async(list_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()

    cfg_names = sorted([n for n in result.result.names if n.startswith("CFG_")])
    node.get_logger().info(f"Found {len(cfg_names)} CFG parameters")

    # Get all values
    get_req = GetParameters.Request()
    get_req.names = cfg_names
    future = get_client.call_async(get_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    values = future.result().values

    # Write YAML
    lines = ["# ZED-X20D configuration dumped from device", "# Use as: ros2 launch ... params_file:=x20d_config.yaml", ""]
    for name, val in zip(cfg_names, values):
        # rcl_interfaces ParameterValue types: 1=bool, 2=int, 3=double, 4=string
        if val.type == 1:
            v = "true" if val.bool_value else "false"
        elif val.type == 2:
            v = str(val.integer_value)
            # Show hex for large integer values (key IDs, baud rates)
            if val.integer_value > 255:
                v = f"0x{val.integer_value:x}"
        elif val.type == 3:
            v = str(val.double_value)
        elif val.type == 4:
            v = f'"{val.string_value}"'
        else:
            v = "0"
        lines.append(f"  {name}: {v}")

    with open(output_file, "w") as f:
        f.write("\n".join(lines) + "\n")

    node.get_logger().info(f"Wrote {len(cfg_names)} parameters to {output_file}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
