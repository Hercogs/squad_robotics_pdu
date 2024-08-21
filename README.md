# squad_robotics_pdu

### Setup on host:
* install socketcand from https://github.com/linux-can/socketcand

This is needed to tunnel CAN interface over to docker when launched with ```network_mode: bridge```.


### To build docker image:
```
docker build --no-cache -t squad_robotics_pdu -f Dockerfile .
```


### To launch docker image:
<b>sudo rights are needed (only once after connecting CAN adapter)!</b>

```
bash start_docker_n_can.bash
```
<b>This script:</b>
* initializes CAN
* starts socketcand server (using docker network interface)
* brings up docker container with ```docker compose up```