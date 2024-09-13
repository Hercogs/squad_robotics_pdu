#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from squad_robotics_pdu.srv import GetBatteryData
import can
import time
from std_msgs.msg import Int8, Int32


class BatteryMonitorService(Node):
    def __init__(self):
        super().__init__("battery_monitor_service")
        self.declare_parameter('robot_name', value='default_robot_name')
        self.robot_name = self.get_parameter("robot_name").value

        self.service = self.create_service(
            GetBatteryData,
            "battery_monitor/get_battery_data",
            self.get_battery_data_callback)

        self.battery_charge_pub = self.create_publisher(msg_type=Int8, topic="battery_charge", qos_profile=10)
        self.battery_current_pub = self.create_publisher(msg_type=Int32, topic="battery_current", qos_profile=10)
        # Publish battery charge in % every 10 seconds
        self.bat_charge_pub_timer = self.create_timer(timer_period_sec=10, callback=self.publish_battery_charge_callback)

        self.bus = can.Bus(
            interface='socketcan',
            channel='can0'
        )

        self.reader = can.BufferedReader()
        self.notifier = can.Notifier(self.bus, [self.reader])
        self.__can_message_ids = (0x100, 0x101, 0x105)


    def __del__(self):
        self.get_logger().info("Shutting down CANBUS")
        self.bus.shutdown()

    def get_battery_data_callback(self, request, response):
        """
        Communicate with battery via CANBUS and return battery status.

        Response has the following fields:
        * `precentage` - remaining battery charge, %
        * `voltage` - battery voltage, V
        * `current` - charge/discharge current, A
        * `full_capacity` - battery full capacity, mAh
        * `remaining_capacity` - battery remaining capacity, mAh
        * `ntc_x` - temperature (x = 1..3), °C

        If a value is invalid, the corresponding field will be:
        * Inf - for float values
        * -1 - for int values
        """
        # init response fields with "not requested" values
        for field, data_type in response.get_fields_and_field_types().items():
            if data_type == 'double':
                setattr(response, field, float('inf'))
            elif data_type == 'int64':
                setattr(response, field, -1)
            elif data_type == 'boolean':
                pass
            else:
                self.get_logger().error(f"Unknown type: {data_type}")
        response.data_valid = True # assume all data is valid at beggining

      
        for msg_id in self.__can_message_ids:

            battery_response = self.__send_CANBUS(msg_id=msg_id)
            if battery_response == None or len(list(battery_response)) == 0:
                response.data_valid = False
                continue
            
            # examine each case
            if msg_id == 0x100:
                response.voltage = int.from_bytes(battery_response[1::-1], byteorder='little', signed=True) / 100 # in Volts
                response.current = int.from_bytes(battery_response[3:1:-1], byteorder='little', signed=True) * 10 / 1000
                response.remaining_capacity = int.from_bytes(battery_response[5:3:-1], byteorder='little', signed=True) *10 #in mAh
            elif msg_id == 0x101:
                response.full_capacity = int.from_bytes(battery_response[1::-1], byteorder='little', signed=True) * 10 # in mAh
                response.cycles = int.from_bytes(battery_response[3:1:-1], byteorder='little', signed=True) # count
                response.precentage = int.from_bytes(battery_response[5:3:-1], byteorder='little', signed=True) # %
            elif msg_id == 0x105:
                response.ntc_1 = round(int.from_bytes(battery_response[1::-1], byteorder='little', signed=True) / 10 - 273.15, 4) # in Celsius
                response.ntc_2 = round(int.from_bytes(battery_response[3:1:-1], byteorder='little', signed=True) / 10 - 273.15, 4) # in Celsius
                response.ntc_3 = round(int.from_bytes(battery_response[5:3:-1], byteorder='little', signed=True) / 10 - 273.15, 4) # in Celsius
            else:
                pass


        return response

    def publish_battery_charge_callback(self):
        """
        Publish battery charge level and current
        """
        battery_response = self.__send_CANBUS(msg_id=0x101)
        if battery_response == None or len(list(battery_response)) == 0:
            return #failed attempt
        
        battery_precentage = self.__combine_bytes(battery_response, 4,2)
        msg = Int8()
        msg.data = battery_precentage
        self.battery_charge_pub.publish(msg)

        # get battery current
        battery_response = self.__send_CANBUS(msg_id=0x100)
        if battery_response == None or len(list(battery_response)) == 0:
            return #failed attempt
        msg = Int32()
        msg.data = int.from_bytes(battery_response[3:1:-1], byteorder='little', signed=True) * 10 # in mAh
        self.battery_current_pub.publish(msg)


    def __combine_bytes(self, data, start_index, num_bytes):
        result = 0
        for i in range(num_bytes):
            result = (result << 8) | data[start_index + i]
        return result

    def __send_CANBUS(self, msg_id:int, response_timeout:float=1) -> can.Message.data:
        """
        Send CANBUS message to battery and return response
        * `msg_id` - CAN message id
        * `response_timeout` - how long to wait for battery's response (in seconds)

        Returns:
        * can.Message.data - if everything is correct
        * None - if failed

        """
        msg = can.Message(
            arbitration_id=msg_id,
            data=[],
            is_extended_id=False
        )
        msg_sent = False
        try:
            self.get_logger().info(f"Sending message with id 0x{msg_id:x} to battery BMS")
            self.bus.send(msg)
            msg_sent = True
        except can.CanError as ce:
            self.get_logger().error(f"CANBUS message with id 0x{msg_id:x} failed to send: {ce}")

        # listen for response
        start_wait_time = time.time()
        battery_response = None
        if msg_sent:
            while True:
                msg = self.reader.get_message() # get message from FIFO

                if time.time() - start_wait_time > response_timeout:
                    self.get_logger().error(f"Did not recieve a response from battery in {response_timeout} seconds")
                    break

                if msg == None:
                    continue

                if msg.arbitration_id == msg_id:
                    # response recieved
                    battery_response = msg.data
                    self.get_logger().info(f"Response for message id 0x{msg_id:x} recieved sucessfully!")
                    break
                
                
        return battery_response



def main():
    rclpy.init()

    battery_monitor_service = BatteryMonitorService()

    rclpy.spin(battery_monitor_service)

    rclpy.shutdown()

if __name__ == "__main__":
    main()