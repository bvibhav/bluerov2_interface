import json
import numpy as np

from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PointStamped
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import UInt16, UInt16MultiArray, Float32

from tf_transformations import quaternion_from_euler


class Publishers:
    def _create_header(self, msg):
        """Create ROS message header"""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_prefix + "/" + self.model_base_link

    def _create_odometry_msg(self, mavdata):
        """Create odometry message from ROV information"""

        # Check if data is available
        if "LOCAL_POSITION_NED" not in mavdata or "ATTITUDE" not in mavdata:
            # print('no LOCAL_POSITION_NED or ATTITUDE data')
            return

        msg = Odometry()

        self._create_header(msg)
        msg.header.frame_id = self.frame_id_prefix + "/odom"
        msg.child_frame_id = self.frame_id_prefix + "/" + self.model_base_link

        # Data from mavlink messages
        local_position_data = mavdata["LOCAL_POSITION_NED"]
        attitude_data = mavdata["ATTITUDE"]

        # Pose
        msg.pose.pose.position = Point(
            x=local_position_data["x"],
            y=local_position_data["y"],
            z=local_position_data["z"],
        )

        msg.twist.twist.linear = Vector3(
            x=local_position_data["vx"] / 100,
            y=local_position_data["vy"] / 100,
            z=local_position_data["vz"] / 100,
        )

        # Orientation
        Q = quaternion_from_euler(
            attitude_data["roll"], attitude_data["pitch"], attitude_data["yaw"]
        )

        msg.pose.pose.orientation = Quaternion(x=Q[0], y=Q[1], z=Q[2], w=Q[3])

        msg.twist.twist.angular = Vector3(
            x=attitude_data["rollspeed"],
            y=attitude_data["pitchspeed"],
            z=attitude_data["yawspeed"],
        )

        return msg

    def _create_imu_msg(self, mavdata):
        """Create imu message from ROV data"""

        # Check if data is available
        if "ATTITUDE" not in mavdata:
            # print('no ATTITUDE data')
            return

        msg = Imu()

        # Add header to message
        self._create_header(msg)

        # http://mavlink.org/messages/common#SCALED_IMU
        imu_data = None
        for i in ["", "2", "3"]:
            try:
                imu_data = mavdata["SCALED_IMU{}".format(i)]
                break
            except Exception as e:
                pass

        if imu_data is None:
            print("no SCALED_IMUX data")
            return

        acc_data = [imu_data["{}acc".format(i)] for i in ["x", "y", "z"]]
        gyr_data = [imu_data["{}gyro".format(i)] for i in ["x", "y", "z"]]
        mag_data = [imu_data["{}mag".format(i)] for i in ["x", "y", "z"]]

        # http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        msg.linear_acceleration.x = acc_data[0] / 100
        msg.linear_acceleration.y = acc_data[1] / 100
        msg.linear_acceleration.z = acc_data[2] / 100

        msg.angular_velocity.x = gyr_data[0] / 1000
        msg.angular_velocity.y = gyr_data[1] / 1000
        msg.angular_velocity.z = gyr_data[2] / 1000

        # http://mavlink.org/messages/common#ATTITUDE
        attitude_data = mavdata["ATTITUDE"]

        # Orientation
        Q = quaternion_from_euler(
            attitude_data["roll"], attitude_data["pitch"], attitude_data["yaw"]
        )

        msg.orientation = Quaternion(x=Q[0], y=Q[1], z=Q[2], w=Q[3])

        # Covriances, TODO: set them in accordance with vehicle specification
        msg.linear_acceleration_covariance = list(0.01 * np.ones(9))
        msg.angular_velocity_covariance = list(0.01 * np.ones(9))
        msg.orientation_covariance = list(0.01 * np.ones(9))

        return msg

    def _create_battery_msg(self, mavdata):
        """Create battery message from ROV data"""

        # Check if data is available
        if "SYS_STATUS" not in mavdata or "BATTERY_STATUS" not in mavdata:
            # print('no SYS_STATUS or BATTERY_STATUS data')
            return

        bat = BatteryState(
            voltage=mavdata["SYS_STATUS"]["voltage_battery"] / 1000,
            curent=mavdata["SYS_STATUS"]["current_battery"] / 100,
            percentage=mavdata["BATTERY_STATUS"]["battery_remaining"] / 100,
        )
        self._create_header(bat)

        return bat

    def _create_raw_msg(self, mavdata):
        if not mavdata:
            return

        return String(data=str(json.dumps(mavdata, ensure_ascii=False)))

    def _create_altitude_msg(self, mavdata):
        """Create altitude message from ROV information"""

        # Check if data is available
        if "RANGEFINDER" not in mavdata:
            # print('no RANGEFINDER data')
            return

        msg = PointStamped(z=mavdata["RANGEFINDER"]["distance"])
        self._create_header(msg)
        msg.header.frame_id = "altitude_link"
        return msg

    def _create_depth_msg(self, mavdata):
        """Create odometry message from ROV information"""

        # Check if data is available
        if "AHRS2" not in mavdata:
            # print('no AHRS2 data')
            return

        if "alt" not in mavdata["AHRS2"]:
            print("no alt data")
            return

        msg = Odometry()

        self._create_header(msg)
        msg.header.frame_id = self.frame_id_prefix + "/odom"
        msg.child_frame_id = self.frame_id_prefix + "/" + self.model_base_link

        msg.pose.pose.position.z = mavdata["VFR_HUD"]["altitude"]
        msg.pose.covariance = np.diag([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]).flatten()

        return msg

    def _create_bottle_pressure_msg(self, mavdata):
        """Create pressure message from ROV information"""

        if "SCALED_PRESSURE" not in mavdata:
            # print('no SCALED_PRESSURE data')
            return

        # Check if data is available
        if "press_abs" not in mavdata["SCALED_PRESSURE"]:
            print("no bottle pressure data")

        return Float32(data=mavdata["SCALED_PRESSURE"]["press_abs"])

    def _create_leak_msg(self):
        """Creates message indicating leak"""
        return

        # Check if data is available
        # if 'severity' not in self.get_data()['STATUSTEXT']:
        #     raise Exception('no leak detection data')

        # msg = UInt16()
        # data = self.get_data()['STATUSTEXT']['severity']
        # msg = self.get_data()['STATUSTEXT']['text']
        # if(data == 2 and msg == "Leak Detected"):
        #     msg.data = 2
        # else:
        #     msg.data = 0
        # self.pub.set_data('/leak_detection', msg)

    """
    def _create_ROV_state(self):
        
        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = (servo_output_raw[6] - 1100) / 8
        #need to check
        camera_angle = servo_output_raw[7] - 1500

        # Create angle from pwm
        camera_angle = -45*camera_angle/400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)

        state = {
            'motor': motor_throttle,
            'light': light_on,
            'camera_angle': camera_angle,
            'mode': mode,
            'arm': arm
        }

        string = String()
        string.data = str(json.dumps(state, ensure_ascii=False))

        self.pub.set_data('/state', string)
    """
