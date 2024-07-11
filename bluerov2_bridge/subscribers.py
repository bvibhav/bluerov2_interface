class Subscribers:
    def _set_mode_callback(self, msg, _):
        """Set ROV mode from topic"""
        self.set_mode(msg.data)

    def _arm_callback(self, msg, _):
        """Set arm state from topic"""

        self.arm_throttle(msg.data)

    def _pwm_callback(self, msg):
        self.set_rc_channels_pwm(list(msg.data))

    def _heartbeat_callback(self, msg):
        self.send_heartbeat()
