#!/bin/bash

echo "delete remap the device serial port(ttyACM*) to ttyPCB"
echo "sudo rm   /etc/udev/rules.d/squad_robotics_pdu.rules"
sudo rm   /etc/udev/rules.d/squad_robotics_pdu.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"