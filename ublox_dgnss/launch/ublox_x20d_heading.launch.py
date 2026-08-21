"""Launch ublox_dgnss_node for ZED-X20D dual-antenna heading configuration."""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for ZED-X20D heading mode."""

    device_serial_string = LaunchConfiguration('device_serial_string')

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="",
        description="Serial string of the ZED-X20D device"
    )

    params_x20d = [
        {'DEVICE_SERIAL_STRING': device_serial_string},
        {'FRAME_ID': "gnss"},

        # Measurement rate: 100ms = 10 Hz (X20D supports up to 20 Hz)
        {'CFG_RATE_MEAS': 0x64},
        {'CFG_RATE_NAV': 0x1},

        # Dynamic model: automotive
        {'CFG_NAVSPG_DYNMODEL': 4},

        # Disable all messages on UART1
        {'CFG_UART1INPROT_NMEA': False},
        {'CFG_UART1INPROT_RTCM3X': False},
        {'CFG_UART1INPROT_UBX': False},
        {'CFG_UART1OUTPROT_NMEA': False},
        {'CFG_UART1OUTPROT_RTCM3X': False},
        {'CFG_UART1OUTPROT_UBX': False},

        # UART2: accept RTCM corrections input (for RTK from base/NTRIP)
        {'CFG_UART2_BAUDRATE': 0x70800},  # 460800
        {'CFG_UART2INPROT_NMEA': False},
        {'CFG_UART2INPROT_RTCM3X': True},
        {'CFG_UART2INPROT_UBX': False},
        {'CFG_UART2OUTPROT_NMEA': False},
        {'CFG_UART2OUTPROT_RTCM3X': False},
        {'CFG_UART2OUTPROT_UBX': False},

        # USB: UBX protocol only (+ RTCM in for corrections via NTRIP client)
        {'CFG_USBINPROT_NMEA': False},
        {'CFG_USBINPROT_RTCM3X': True},
        {'CFG_USBINPROT_UBX': True},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_USBOUTPROT_RTCM3X': False},
        {'CFG_USBOUTPROT_UBX': True},

        # Key messages for heading and position
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_DOP_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_SAT_USB': 0x5},
        {'CFG_MSGOUT_UBX_NAV_SIG_USB': 0x5},
        {'CFG_MSGOUT_UBX_NAV_VELNED_USB': 0x1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_EOE_USB': 0x1},
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

    container_navsatfix = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                namespace='gnss',
                name='ublox_nav_sat_fix_hp'
            )
        ]
    )

    return launch.LaunchDescription([
        log_level_arg,
        device_serial_string_arg,
        container_x20d,
        container_navsatfix,
    ])
