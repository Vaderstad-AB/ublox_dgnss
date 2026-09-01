"""Launch ublox_dgnss_node for ZED-X20D basic position + heading test with NTRIP RTK."""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for ZED-X20D basic test with NTRIP corrections."""

    device_serial_string = LaunchConfiguration('device_serial_string')

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="",
        description="Serial string of the ZED-X20D device"
    )
    ntrip_host_arg = DeclareLaunchArgument(
        "ntrip_host", default_value="caster.centipede.fr",
        description="NTRIP caster host"
    )
    ntrip_port_arg = DeclareLaunchArgument(
        "ntrip_port", default_value="2101",
        description="NTRIP caster port"
    )
    ntrip_mountpoint_arg = DeclareLaunchArgument(
        "ntrip_mountpoint", default_value="VaderstadAB",
        description="NTRIP mountpoint"
    )
    ntrip_username_arg = DeclareLaunchArgument(
        "ntrip_username", default_value="centipide",
        description="NTRIP username"
    )
    ntrip_password_arg = DeclareLaunchArgument(
        "ntrip_password", default_value="centipide",
        description="NTRIP password"
    )

    params_x20d = [
        {'DEVICE_SERIAL_STRING': device_serial_string},
        {'FRAME_ID': "gnss"},

        # 1 Hz for basic testing
        {'CFG_RATE_MEAS': 0x3e8},
        {'CFG_RATE_NAV': 0x1},

        # USB: UBX + RTCM input (for corrections from NTRIP)
        {'CFG_USBINPROT_NMEA': False},
        {'CFG_USBINPROT_RTCM3X': True},
        {'CFG_USBINPROT_UBX': True},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_USBOUTPROT_RTCM3X': False},
        {'CFG_USBOUTPROT_UBX': True},

        # Essential messages for position and heading
        # Per Ardusimple guide: all heading output available under UBX-NAV-DAHEADING
        {'CFG_MSGOUT_UBX_NAV_DAHEADING_USB': 0x1},  # Dual antenna heading
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
        {'CFG_MSGOUT_UBX_RXM_COR_USB': 0x1},
    ]

    ntrip_params = [
        {'use_https': False},
        {'host': LaunchConfiguration('ntrip_host')},
        {'port': 2101},
        {'mountpoint': LaunchConfiguration('ntrip_mountpoint')},
        {'username': LaunchConfiguration('ntrip_username')},
        {'password': LaunchConfiguration('ntrip_password')},
        {'log_level': "INFO"},
        {'maxage_conn': 30},
    ]

    container_x20d = ComposableNodeContainer(
        name='ublox_dgnss_x20d',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace='gnss',
                parameters=params_x20d
            )
        ]
    )

    ntrip_container = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ntrip_client_node',
                plugin='ublox_dgnss::NTRIPClientNode',
                name='ntrip_client',
                parameters=ntrip_params
            )
        ]
    )

    return launch.LaunchDescription([
        log_level_arg,
        device_serial_string_arg,
        ntrip_host_arg,
        ntrip_port_arg,
        ntrip_mountpoint_arg,
        ntrip_username_arg,
        ntrip_password_arg,
        container_x20d,
        ntrip_container,
    ])
