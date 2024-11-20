# squad_robotics_pdu

This package manages Vikings Bot Power Distribution Unit (PDU) and handles communication with Battery Management System (BMS) via CANBUS. Package has been developed and tested on ROS2 Humble and Ubuntu 22.04 LTS.



## Setup

### Setup Udev rules for PDU and CANBUS adapter:
```
sudo bash udev/create_udev_rules.sh
```
Rules can be deleted by executing ```delete_udev_rules.sh``` script the same way.



### A) Build docker image

1. __Build image__
    ```
    docker build --no-cache -t squad_robotics_pdu .
    ```

2. __Start Docker container__
    ```
    docker compose up -d
    ```

### B) Build package on host

1. __Install depenencies__
    ```
    pip install -r requirements.txt
    ```

2. __Install PCB Bridge__
    ```
    cd PDU_ros2_node && sudo apt install ./ros-humble-robot-*jammy_amd64.deb && cd ..
    ```
    In case of problems, see ```PDU_ros2_node/PCB bridge setup 25032024.pdf```.
3. __Source ROS2 and build package__
    ```
    . /opt/ros/humble/setup.bash && colcon build --packages-select squad_robotics_pdu
    ```

4. __Launch package__
    ```
    . install/setup.bash && ros2 launch squad_robotics_pdu launch_pdu.launch.py
    ```


## Nodes and parameters
### Parameters
The main launch file ```launch_pdu.launch.py``` accepts the following parameter:

* __```vikings_bot_name```__ - Set namespace for nodes and topics.

### Nodes
* __```pcb_ros2_bridge```__ - Handles communication with PDU.
                
    * Refer to ```documents/PC to LCm message.pdf``` and other files in ```documents``` to learn more about communication with PDU.

* __```pcb_configure```__ - Upload configuration to PDU.
    
    * See ```config/config.json``` to add functionalities to PDU. Examine ```scripts/upload_config.py``` to see what kind of configuration can be done.

* __```battery_monitor```__ - Communicate with battery BMS.

    * __Published topics:__
    
        * ```battery_charge``` - Publish current battery charge %.
        * ```battery_current``` - Publish battery current in mA.

    * __Service:__

        * ```battery_monitor/get_battery_data``` - Get all battery data (see ```srv/GetBatteryData.srv```).

* __```input_state_publisher```__ - If any boolean inputs are configured, this publishes their status.

    * __Published topic:__
        
        * ```input_states``` - Publish boolean input states in json string.
