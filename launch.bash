#!bin/bash

# This script is executed when docker container is brought up

# create a virtual can0 interface in docker container
echo "Setting up vcan."
ip link add vcan0 type vcan bitrate 250000

# # bring up the interface
ip link set vcan0 up
echo "vcan setup completed."

# launch socketcand client (using docker default ip)
echo "Starting socketcand client..."
nohup socketcandcl -s 172.17.0.1 -i can0,vcan0 -v &

# start ros2 nodes
ros2 launch squad_robotics_pdu launch_pdu.launch.py &

wait -n

exit $?
