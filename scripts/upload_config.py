#! /usr/bin/env python3

import os
import argparse
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random

from ament_index_python.packages import get_package_share_directory

class configurePcbNode(Node):
    def __init__(self, filename=None):
        super().__init__('pcb_configure')

        self.get_logger().info(f"Node {self.get_name()} started!")

        #time.sleep(3)  # Sleep to wait for subscriber node

        self.__package_name = "squad_robotics_pdu"
        self.__config_filename = "config.json"

        self.publisher_ = self.create_publisher(String, '/f2pcb', 10)
        self.subscription = self.create_subscription( String, '/pcb2f', self.listener_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_messages)
        self.sendOut = []
        self.gotMessages = []

        self.lastCmd = {}
        self.lastCmdID = 0
        self.lastCmdTime = 0
        self.lastCmdRetry = 0

        self.get_logger().info(f"Node {self.get_name()} configured!")

        self.generate_messages()


    def listener_callback(self, msg):
        self.get_logger().info(f"listener_callback(): {msg.data}")
        self.gotMessages.append(json.loads(msg.data))

    def got_response(self):
        if self.lastCmd:
            for gm in self.gotMessages:
                if "ID" in gm:
                    if gm["ID"] == self.lastCmdID:
                        return 1
        else:
            return 1
        return 0

    def generate_messages(self):
        
        file_path = os.path.join(
            get_package_share_directory(self.__package_name),
            "config",
            self.__config_filename
        )

        try:
            with open(file_path) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} not found.")
            return []

        #messages = []
        for item in data:
            if 'functionality' not in item or 'name' not in item['functionality']:
                continue

            functionality = item['functionality']

            raw = {"functionality": {}}
            raw["functionality"]["cmd"] = "add"
            raw["functionality"]["functionalityName"] = functionality['name']
            self.sendOut.append(raw["functionality"])

            # Generate messages for outputs
            if 'outputs' in functionality:
                for output_pin in functionality['outputs']:
                    raw = {"functionality": {}}
                    raw["functionality"]["cmd"] = "addPin"
                    raw["functionality"]["functionalityName"] = functionality['name']
                    raw["functionality"]["pinTypeOutput"] = True
                    raw["functionality"]["pinNumber"] = output_pin
                    self.sendOut.append(raw["functionality"])

            # Generate messages for inputs
            if 'inputs' in functionality:
                for input_pin in functionality['inputs']:
                    raw = {"functionality": {}}
                    raw["functionality"]["cmd"] = "addPin"
                    raw["functionality"]["functionalityName"] = functionality['name']
                    raw["functionality"]["pinTypeOutput"] = False
                    raw["functionality"]["pinNumber"] = input_pin
                    self.sendOut.append(raw["functionality"])

            # Generate message for limits
            if 'limits' in functionality and isinstance(functionality['limits'], (list, tuple)) and len(functionality['limits']) == 3:
                raw = {"functionality": {}}
                raw["functionality"]["cmd"] = "setCurrentLimit"
                raw["functionality"]["functionalityName"] = functionality['name']
                raw["functionality"]["minCurrent"] = functionality['limits'][0]
                raw["functionality"]["warningCurrent"] = functionality['limits'][1]
                raw["functionality"]["faultCurrent"] = functionality['limits'][2]
                self.sendOut.append(raw["functionality"])

            if 'outputsAsPWM' in functionality and functionality['outputsAsPWM'] == True:
                raw = {"functionality": {}}
                raw["functionality"]["cmd"] = "setOutputsAsPWM"
                raw["functionality"]["functionalityName"] = functionality['name']
                self.sendOut.append(raw["functionality"])

            if 'setOverCurrentIntegralLimit' in functionality:
                raw = {"functionality": {}}
                raw["functionality"]["cmd"] = "setOverCurrentIntegralLimit"
                raw["functionality"]["functionalityName"] = functionality['name']
                raw["functionality"]["overCurrentIntegralMax"] = functionality['setOverCurrentIntegralLimit']
                self.sendOut.append(raw["functionality"])
            # quick way to set default level of pin
            if 'enabled' in functionality:
                raw = {"functionality": {}}
                raw["functionality"]["cmd"] = "setOutputByName"
                raw["functionality"]["functionalityName"] = functionality['name']
                raw["functionality"]["outputLevel"] = int(functionality['enabled'])
                self.sendOut.append(raw["functionality"])                

    def publish_messages(self):
        message = {}
        if self.got_response() == 1:
            if self.sendOut:
                message = self.sendOut.pop(0)
                self.lastCmdID = random.randint(1, 9000)
                message["ID"] = self.lastCmdID
                self.lastCmdRetry = 0
                self.lastCmdTime = time.time()
        else:
            if time.time() > self.lastCmdTime + 2:
                self.lastCmdTime = time.time()
                self.lastCmdRetry = self.lastCmdRetry + 1
                message = self.lastCmd
                 
        if message:
            self.lastCmd = message
            msg = String()
            msg.data = json.dumps(message)
            self.lastCmdTime = time.time()
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    configure_node = configurePcbNode()
    
    rclpy.spin(configure_node)
    configure_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()