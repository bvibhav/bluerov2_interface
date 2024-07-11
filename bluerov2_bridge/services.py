from std_srvs.srv import SetBool
from bluerov2_interface.srv import SetMode


class Services:
    def _service_arm(self, request, response):
        self.arm_throttle(request.data)

        self.get_logger().info("ARM_DISARM ROS Service complete")
        response.success = True
        return response

    def _service_setmode(self, request, response):
        self.set_mode(request.data)

        self.get_logger().info("DO_SET_MODE ROS Service complete")
        response.success = True
        return response


class ClientProxies:
    def __init__(self):
        self._client_arm = self.create_client(SetBool, "/bluerov2/arm")
        self._client_setmode = self.create_client(SetMode, "/bluerov2/setmode")

    def _mode_manual(self):
        self.get_logger().info("Setting MANUAL mode")
        self._client_setmode.call_async(SetMode.Request(data="manual"))

    def _mode_stabilize(self):
        self.get_logger().info("Setting STABILIZE mode")
        self._client_setmode.call_async(SetMode.Request(data="stabilize"))

    def _mode_poshold(self):
        self.get_logger().info("Setting POSHOLD mode")
        self._client_setmode.call_async(SetMode.Request(data="poshold"))

    def _mode_althold(self):
        self.get_logger().info("Setting ALT_HOLD mode")
        self._client_setmode.call_async(SetMode.Request(data="alt_hold"))

    def _arm(self):
        self.get_logger().info("Arming thursters")
        self._client_arm.call_async(SetBool.Request(data=True))

    def _disarm(self):
        self.get_logger().info("Disaraming thursters")
        self._client_arm.call_async(SetBool.Request(data=False))
