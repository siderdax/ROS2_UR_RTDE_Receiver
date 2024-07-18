from time import sleep
from typing import List
from threading import Thread
import rclpy
from rclpy import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from std_msgs.msg import String
from rtde_interfaces.msg import UrRtde

import json

from .rtde import rtde
from .rtde import rtde_config


class UrRtdeReceiver(Node):
    con: rtde.RTDE = None
    receive_thread: Thread = None
    is_running: bool = False
    is_json_out = False

    def __init__(self):
        super().__init__("ur_rtde_receiver")
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.declare_parameter("ur_rtde_env.host", "localhost")
        self.declare_parameter(
            "ur_rtde_env.config",
            "install/ur_rtde_receiver/share/ur_rtde_receiver/config/configuration.xml",
        )
        self.declare_parameter("ur_rtde_env.frequency", 10)
        self.declare_parameter("ur_rtde_env.json", False)

        self.is_json_out = self.get_parameter("ur_rtde_env.json").value

        if self.is_json_out == True:
            self.publisher = self.create_publisher(String, "ur_rtde_data", 10)
        else:
            self.publisher = self.create_publisher(UrRtde, "ur_rtde_data", 10)

        self.connect()

    def on_receiving_thread(self):
        while self.is_running:
            try:
                state = self.con.receive()

                if state is not None:
                    if self.publisher.msg_type == UrRtde:
                        msg = UrRtde()
                        msg.timestamp = state.timestamp
                        msg.target_q = state.target_q
                        msg.target_qd = state.target_qd
                        msg.target_qdd = state.target_qdd
                        msg.target_current = state.target_current
                        msg.target_moment = state.target_moment
                        msg.actual_q = state.actual_q
                        msg.actual_qd = state.actual_qd
                        msg.actual_current = state.actual_current
                        msg.joint_control_output = state.joint_control_output
                        msg.actual_tcp_pose = state.actual_TCP_pose
                        msg.actual_tcp_speed = state.actual_TCP_speed
                        msg.actual_tcp_force = state.actual_TCP_force
                        msg.target_tcp_pose = state.target_TCP_pose
                        msg.target_tcp_speed = state.target_TCP_speed
                        msg.actual_digital_input_bits = state.actual_digital_input_bits
                        msg.joint_temperatures = state.joint_temperatures
                        msg.actual_execution_time = state.actual_execution_time
                        msg.robot_mode = state.robot_mode
                        msg.joint_mode = state.joint_mode
                        msg.safety_status = state.safety_status
                        msg.actual_tool_accelerometer = state.actual_tool_accelerometer
                        msg.speed_scaling = state.speed_scaling
                        msg.target_speed_fraction = state.target_speed_fraction
                        msg.actual_momentum = state.actual_momentum
                        msg.actual_main_voltage = state.actual_main_voltage
                        msg.actual_robot_voltage = state.actual_robot_voltage
                        msg.actual_robot_current = state.actual_robot_current
                        msg.actual_joint_voltage = state.actual_joint_voltage
                        msg.actual_digital_output_bits = state.actual_digital_output_bits
                        msg.runtime_state = state.runtime_state
                        msg.joint_position_deviation_ratio = (
                            state.joint_position_deviation_ratio
                        )
                        self.publisher.publish(msg)
                    elif self.publisher.msg_type is String:
                        msg = String()
                        msg.data = json.dumps(state.__dict__)
                        self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(str(e))
                self.disconnect()

                if self.is_running == True:
                    self.connect()
                pass

    def on_set_parameters(self, params: List[Parameter]):
        return SetParametersResult(successful=True)

    def connect(self):
        while self.con == None or self.con.is_connected() == False:
            try:
                if self.con != None and self.con.is_connected() == True:
                    self.disconnect()

                self.con = rtde.RTDE(self.get_parameter("ur_rtde_env.host").value)
                conf = rtde_config.ConfigFile(
                    self.get_parameter("ur_rtde_env.config").value
                )
                output_names, output_types = conf.get_recipe("out")
                self.con.connect()

                # get controller version
                self.con.get_controller_version()

                # setup recipes
                if not self.con.send_output_setup(
                    output_names,
                    output_types,
                    frequency=self.get_parameter("ur_rtde_env.frequency").value,
                ):
                    self.get_logger().error("Unable to configure output")

                # start data synchronization
                if not self.con.send_start():
                    self.get_logger().error("Unable to start synchronization")

                self.receive_thread = Thread(target=self.on_receiving_thread)
                self.is_running = True
                self.receive_thread.start()
                self.get_logger().info("Connected to host")
            except Exception as e:
                self.get_logger().error(str(e))
                self.get_logger().warn("Reconnect to host")
                self.disconnect()
                sleep(1)
                pass

    def disconnect(self):
        try:
            self.con.send_pause()
        except:
            pass

        try:
            self.con.disconnect()
            self.is_running = False
            self.receive_thread.join()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    ur_rtde_receiver = UrRtdeReceiver()
    rclpy.spin(ur_rtde_receiver)
    ur_rtde_receiver.disconnect()
    ur_rtde_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
