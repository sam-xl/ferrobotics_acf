#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import messages and services
from std_msgs.msg import Float32
from stamped_std_msgs.msg import Float32Stamped
from ferrobotics_acf.srv import SetFloat, SetDuration
from ferrobotics_acf.msg import ACFTelem, ACFTelemStamped
from sensor_msgs.msg import JointState

import socket
import sys

bytes_args = ()
if sys.version_info.major >= 3:
    bytes_args = ("ASCII",)


class FerroboticsACF(Node):
    DEFAULT_IP = "192.168.99.1"
    DEFAULT_PORT = 7070
    DEFAULT_AUTHENTICATION = "ferba"
    DEFAULT_ID = 1040
    DEFAULT_F_MAX = 100
    TERMINATOR = "k"
    DISCONNECT = "c"
    BUFSIZE = 128
    DELIMINATOR = " "
    DEFAULT_TIMEOUT = 5
    MAX_T_RAMP = 10
    PAYLOAD_SHARE = 0.1
    ERROR_CODE_BIN_LENGTH = 8
    ERROR_MESSAGES = [
        "valve error",
        "sensor error",
        "Head and controller are not compatible",
        "set_f_target cannot be reached",
        "set_f_zero set to 0",
        "one or more input values out of range",
        "INCAN no communication",
        "Reading of INCAN parameters not complete",
    ]

    def __init__(self):
        super().__init__('acf')
        self.get_params()
        self.connect()
        assert self.authenticate(), "Failed to authenticate"
        self.ros_setup()
        self.get_logger().info(f"Done initialising.")

    def get_params(self):
        self.declare_parameter('ip', self.DEFAULT_IP)
        self.declare_parameter('port', self.DEFAULT_PORT)
        self.declare_parameter('authentication', self.DEFAULT_AUTHENTICATION)
        self.declare_parameter('id', self.DEFAULT_ID)
        self.declare_parameter('f_max', self.DEFAULT_F_MAX)
        self.declare_parameter('initial_force', 0.0)
        self.declare_parameter('ramp_duration', 0.0)
        self.declare_parameter('payload', 0.0)
        self.declare_parameter('frequency', 0)
        self.declare_parameter('joint_name', "")

        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.authentication = self.get_parameter('authentication').get_parameter_value().string_value
        self.id = self.get_parameter('id').get_parameter_value().integer_value
        self.f_max = self.get_parameter('f_max').get_parameter_value().integer_value
        self.initial_force = self.get_parameter('initial_force').get_parameter_value().double_value
        assert self.check_force(self.initial_force), "Invalid force value"
        self.ramp_duration = self.get_parameter('ramp_duration').get_parameter_value().double_value
        assert self.check_ramp_duration(self.ramp_duration), "Invalid ramp duration"
        self.payload = self.get_parameter('payload').get_parameter_value().double_value
        assert self.check_payload(self.payload), "Invalid payload value"
        self.frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.force = 0

    def check_force(self, force):
        return -self.f_max <= force <= self.f_max

    def check_ramp_duration(self, duration):
        return 0 <= duration <= self.MAX_T_RAMP

    def check_payload(self, payload):
        return 0 <= payload <= self.PAYLOAD_SHARE * self.f_max

    def connect(self):
        self.get_logger().info(f"Attempting to connect to {self.ip}:{self.port}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        self.get_logger().info(f"Connected.")

    def authenticate(self):
        data = bytes(self.authentication + self.TERMINATOR, *bytes_args)
        self.sock.send(data)
        return data == self.sock.recv(self.BUFSIZE)

    def send_command(self, force):
        try:
            self.sock.send(
                bytes(
                    self.DELIMINATOR.join(
                        str(field)
                        for field in (
                            self.id,
                            force,
                            self.initial_force,
                            self.ramp_duration,
                            self.payload,
                            0,
                        )
                    )
                    + self.TERMINATOR,
                    *bytes_args
                )
            )
        except:
            self.get_logger().warn("Reconnecting...")
            self.connect()
            assert self.authenticate(), "Failed to authenticate"
            self.send_command(force)

    def disconnect(self):
        try:
            self.sock.send(self.DISCONNECT)
        except:
            pass

    def timer_callback(self):
        self.send_command(self.force)
        self.initial_force = self.force
        telem = self.handle_telem()
        if self.joint_state_pub is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [telem.position / 1e3] # Convert from mm to m
            # msg.velocity = [] # TODO Calculate this by finite difference
            # msg.effort = [] # TODO Send the current force at the joint
            self.joint_state_pub.publish(msg)

    def command_handler(self, msg):
        self.force = msg.data
        if self.frequency <= 0:
            self.timer_callback()

    def handle_telem(self) -> ACFTelem:
        data, stamp = self.recv_telem()
        telem_stamped = ACFTelemStamped()
        telem_stamped.header.stamp = stamp
        telem = ACFTelem()
        telem.id = int(data[0])
        telem.force = float(data[1])
        telem.position = float(data[2])
        telem.in_contact = bool(data[3])
        telem.in_error = int(data[4]) > 0
        telem.errors = [
            bool(int(data[4]) & (1 << i)) for i in range(self.ERROR_CODE_BIN_LENGTH)
        ]
        telem.error_messages = [
            self.ERROR_MESSAGES[i] for i, error in enumerate(telem.errors) if error
        ]
        telem_stamped.telemetry = telem
        self.telem_pub.publish(telem_stamped)
        return telem

    def recv_telem(self):
        telem_time_msg = self.get_clock().now().to_msg()
        return (
            self.sock.recv(self.BUFSIZE).decode().strip(self.TERMINATOR).split(self.DELIMINATOR), telem_time_msg
        )

    def set_payload(self, request : SetFloat.Request, response : SetFloat.Response):
        if not self.check_payload(request.value):
            response.success = False
            response.message = "Invalid payload value."
        else:
            response.success = True
            response.message = ""
            self.payload = request.value
        return response

    def set_f_zero(self, request : SetFloat.Request, response : SetFloat.Response):
        if not self.check_force(request.value):
            response.success = False
            response.message = "Invalid force value."
        else:
            response.success = True
            response.message = ""
            self.initial_force = request.value
        return response

    def set_t_ramp(self, request : SetDuration.Request, response : SetDuration.Response):
        duration = request.duration.sec + (request.duration.nanosec / 1e9)
        if not self.check_ramp_duration(duration):
            response.success = False
            response.message = "Invalid duration."
        else:
            response.success = True
            response.message = ""
            self.ramp_duration = duration
        return response

    def ros_setup(self):
        self.create_subscription(Float32Stamped, '~/force', self.command_handler, 10)
        self.telem_pub = self.create_publisher(ACFTelemStamped, '~/telem', 5)
        self.create_service(SetFloat, '~/set_payload', self.set_payload)
        self.create_service(SetFloat, '~/set_f_zero', self.set_f_zero)
        self.create_service(SetDuration, '~/set_t_ramp', self.set_t_ramp)
        if self.frequency > 0:
            self.create_timer(1.0 / self.frequency, self.timer_callback)
        if self.joint_name != "":
            self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        else:
            self.joint_state_pub = None

if __name__ == "__main__":
    rclpy.init()
    acf_node = FerroboticsACF()
    rclpy.spin(acf_node)
    acf_node.destroy_node()
    rclpy.shutdown()
