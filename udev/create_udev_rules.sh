#!/bin/bash

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

cd "$parent_path"

for f in /usr/lib/udev/rules.d/*brltty*.rules; do
    sudo ln -sf /dev/null "/etc/udev/rules.d/$(basename "$f")"
done

echo "remap the device serial port(ttyUSB*) to  ttyPCB"
echo "PDU usb connection as /dev/ttyPCB , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy squad_robotics_pdu.rules to  /etc/udev/rules.d/"
sudo cp ./squad_robotics_pdu.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "