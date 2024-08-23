#!/bin/bash

#########################################
# USE THIS INSTEAD OF docker compose up #
#########################################

# start socketcand server in background
socketcand -v -i can0 -l docker0 &

# start docker compose
docker compose up

wait -n

exit $?
