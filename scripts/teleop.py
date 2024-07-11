#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16MultiArray, Bool

from bluerov2_bridge.services import ClientProxies

import signal

map_axes = {
    "LX": 0,
    "LY": 1,
    "LT": 2,
    "RX": 3,
    "RY": 4,
    "RT": 5,
    "DX": 6,
    "DY": 7,
}

map_buttons = {
    "A": 0,  # Button A
    "B": 1,  # Button B
    "X": 2,  # Button X
    "Y": 3,  # Button Y
    "LB": 4,  # Left tigger button
    "RB": 5,  # Right trigger button
    "BK": 6,  # Back button
    "ST": 7,  # Start button
    "LG": 8,  # Logitech center button
    "LJ": 9,  # Left joystick press
    "RJ": 10,  # Right joystick press
}


def axID(key):
    return map_axes[key]


def btID(key):
    return map_buttons[key]


class Teleop(Node, ClientProxies):
    """
    # BlueROV modes from mavlink
    ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    """

    def __init__(self, node_name):
        # Explicit inheritence as super doesn't work with Node class **kwargs
        Node.__init__(self, node_name=node_name, namespace="bluerov2")
        ClientProxies.__init__(self)

        self.frame_id_prefix = self.get_namespace()[1:]
        self.model_base_link = "base_link"

        # Joystick subscriber
        self.create_subscription(Joy, "/joy", self._callback, 1)

        # Publishers
        self.pub_pwm = self.create_publisher(UInt16MultiArray, "set_pwm", 1)
        self.pub_beat = self.create_publisher(Bool, "heartbeat", 1)

        # Button ID to function mapping
        self.fcns = {
            "0": self._mode_poshold,  # A
            "1": self._mode_althold,  # B
            "2": self._mode_stabilize,  # X
            "3": self._mode_manual,  # Y
            "4": self._notused,  # LB
            "5": self._toggle_lights,  # RB
            "6": self._disarm,  # Back
            "7": self._arm,  # Start
            "8": self._notused,  # Logi
            "9": self._notused,  # LJoy
            "10": self._notused,  # RJoy
        }

        # Default light pwm
        self.pwm_lights = 1100

        # Using to avoid multiple registration of buttons
        self.buttons = [0]*11

        # Publish heartbeat timer, NOTE: might not be required if Joystick publisher continously.
        self.timer = self.create_timer(1, self._send_heartbeat)

    def _send_heartbeat(self):
        self.pub_beat.publish(Bool(data=True))

    def _notused(self):
        print("This button is not used")

    def _toggle_lights(self):
        self.pwm_lights += 200
        if self.pwm_lights > 1900:
            self.pwm_lights = 1100
        print("Lights PWM:", self.pwm_lights)

    def calc_pwm(self, val, pwm_min=1100, pwm_max=1900):
        """returns pwm value from -1 to 1 input"""
        mult = (pwm_max - pwm_min) / 2
        base = pwm_min + mult
        return int(base + mult * val)

    def _callback(self, msg):
        # Handling button presses. NOTE: ROS2 joy node publishes continuosly, unlike ROS1.
        for k in range(0, len(msg.buttons)):
            # Pressed state
            if msg.buttons[k] and not self.buttons[k]:
                self.buttons[k] = msg.buttons[k]
                self.fcns[str(k)]()

            # Released state
            elif not msg.buttons[k] and self.buttons[k]:
                self.buttons[k] = msg.buttons[k]

        # Handling Joy Axes
        pwm_msg = UInt16MultiArray()
        pwm_msg.data = [65535 for _ in range(16)]
        pwm_msg.data[0] = self.calc_pwm(-msg.axes[axID("DY")])  # Pitch
        pwm_msg.data[1] = self.calc_pwm(-msg.axes[axID("DX")])  # Roll
        pwm_msg.data[2] = self.calc_pwm(msg.axes[axID("RY")])  # throttle
        pwm_msg.data[3] = self.calc_pwm(-msg.axes[axID("RX")])  # Yaw
        pwm_msg.data[4] = self.calc_pwm(msg.axes[axID("LY")])  # Forward
        pwm_msg.data[5] = self.calc_pwm(-msg.axes[axID("LX")])  # Lateral
        pwm_msg.data[8] = self.pwm_lights  # Lights

        self.pub_pwm.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)

    teleop = Teleop(node_name="bluerov2_teleop")

    # Using signal handler to shutdown thrusters. NOTE: shutdown hooks seems to missing from ROS2
    def signal_handler(frame, sig):
        teleop._disarm()

        # Destroy the node explicitly, otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        teleop.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # Spin the node
    rclpy.spin(teleop)


if __name__ == "__main__":
    main()
