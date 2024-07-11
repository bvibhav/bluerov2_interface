#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from bluerov2_bridge.bridge import Bridge
from bluerov2_bridge.publishers import Publishers
from bluerov2_bridge.subscribers import Subscribers
from bluerov2_bridge.services import Services

from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16, UInt16MultiArray, Float32

from std_srvs.srv import SetBool
from bluerov2_interface.srv import SetMode

import json, signal


class Bluerov2(Node, Bridge, Publishers, Subscribers, Services):
    def __init__(self, node_name, device, baudrate):
        # Explicit inheritence as super doesn't work with Node class **kwargs
        Node.__init__(self, node_name=node_name, namespace="bluerov2")
        Bridge.__init__(self, device, baudrate)

        self.frame_id_prefix = self.get_namespace()[1:]
        self.model_base_link = "base_link"

        # Placeholders to keep code clean
        gen_pub = self.create_publisher
        gen_sub = self.create_subscription

        # Publishers
        self.pubs = [
            [gen_pub(String, "raw", 1), self._create_raw_msg],
            [gen_pub(Imu, "imu", 1), self._create_imu_msg],
            [gen_pub(Odometry, "odometry", 1), self._create_odometry_msg],
            [gen_pub(BatteryState, "battery", 1), self._create_battery_msg],
            [gen_pub(PointStamped, "altitude", 1), self._create_altitude_msg],
            [gen_pub(Float32, "bottle_pressure", 1), self._create_bottle_pressure_msg],
            # [gen_pub(Odometry, 'depth', 1), self._create_depth_msg],
            # [gen_pub(UInt16, 'leak_detection', 1), self._create_leak_msg],
            # [gen_pub(Image, 'camera/image_raw', 1), self._create_camera_msg],
            # [gen_pub(String, 'state', 1), self._create_ROV_state],
        ]

        # Subscribers
        gen_sub(UInt16MultiArray, "set_pwm", self._pwm_callback, 1),
        gen_sub(Bool, 'heartbeat', self._heartbeat_callback, 1),
        # gen_sub(UInt16MultiArray, 'setpoint/pwm', self._setpoint_pwm_callback, 1),

        # Services
        self.create_service(SetBool, "arm", self._service_arm)
        self.create_service(SetMode, "setmode", self._service_setmode)

        # Publish everything
        self.timer = self.create_timer(1 / 50, self.publish)

        # Making sure that motors are disarmed upon start
        self.send_heartbeat()
        # self.arm_throttle(False)

        # self._arm()
        # self._disarm()
        # self._mode_poshold()

    def publish(self):
        # print(json.dumps(mavdata['COMMAND_ACK'], indent=2))

        mavdata = self.update()
        for pub, msg_creator in self.pubs:
            msg_pub = msg_creator(mavdata)
            if msg_pub is not None:
                pub.publish(msg_pub)


def main(args=None):
    rclpy.init(args=args)

    bluerov = Bluerov2(
        node_name="bluerov2_interface", device="udp:127.0.0.1:14550", baudrate=115200
    )

    # Using signal handler to shutdown thrusters. NOTE: shutdown hooks seems to missing from ROS2
    def signal_handler(frame, sig):
        bluerov.arm_throttle(False)
        bluerov.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(bluerov)

    # Destroy the node explicitly, otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # bluerov.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
