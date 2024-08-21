#!/bin/bash

#########################################
# USE THIS INSTEAD OF docker compose up #
#########################################


# make sure the can0 interface is set up
sudo ip link set can0 up type can bitrate 250000

# start socketcand server in background
socketcand -v -i can0 -l docker0 &

# start docker compose
docker compose up

wait -n

exit $?
