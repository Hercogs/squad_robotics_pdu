#!/usr/bin/env python3

import rclpy
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from ament_index_python.packages import get_package_share_directory


class InputStatePublisher(Node):
    """
    Publishes status of PDU inputs in JSON format.
    """
    def __init__(self):
        super().__init__("input_state_publisher")

        self.input_status:dict = {}
        self.init_inputs()

        self.pdu_subscriber = self.create_subscription(msg_type=String, topic="/pcb2f", callback=self.clb_pcb2f, qos_profile=10)
        self.pub_input_state = self.create_publisher(msg_type=String, topic="input_states", qos_profile=10)
        self.pub_status_query = self.create_publisher(msg_type=String, topic="/f2pcb", qos_profile=10)

        self.status_query_timer = self.create_timer(timer_period_sec=0.2, callback=self.query_pdu)


    def clb_pcb2f(self, msg:String):
        """
        Update input status
        """
        if not self.input_status:
            return
        
        msg_json = json.loads(msg.data)

        exec_status = msg_json["EXEC"]

        if exec_status != "OK" or type(exec_status) != dict:
            return
        
        if "RES" not in exec_status.keys():
            return

        res = msg_json["RES"]
        res = res.split(" ")
        functionality_name = res[0]
        functionality_status = res[2] == "HIGH"

        if functionality_name in self.input_status.keys():
            self.input_status[functionality_name] = functionality_status

        # Publish updated input status
        new_msg = String()
        new_msg.data = json.dumps(self.input_status)
        self.pub_input_state.publish(new_msg)

    def query_pdu(self):
        """
        Get status of inputs from PDU
        """
        if not self.input_status:
            return
        
        for functionality in self.input_status.keys():
            message_data = {
                'cmd': 'getStateByName',
                'functionalityName':functionality
            }
            msg = String()
            msg.data = json.dumps(message_data)
            self.pub_status_query.publish(msg)

    def init_inputs(self):
        """
        Read config file and find input pins.
        """
        package_name = "squad_robotics_pdu"
        config_filename = "config.json"
        file_path = os.path.join(
            get_package_share_directory(package_name),
            "config",
            config_filename
        )
        data = None
        try:
            f = open(file_path, "r")
            data = json.load(f)
            f.close()
        except Exception as e:
            self.get_logger().error(f"Couldn't open file: {e}")
            self.destroy_node()

        for item in data:
            if 'functionality' not in item or 'name' not in item['functionality']:
                continue

            functionality = item['functionality']
            
            if 'inputs' in functionality:
                # Assume each functionality has excatly 1 input pin
                self.input_status[functionality["name"]] = False

def main():
    rclpy.init()

    input_state_publisher = InputStatePublisher()

    rclpy.spin(input_state_publisher)

    rclpy.shutdown()

if __name__ == "__main__":
    main()