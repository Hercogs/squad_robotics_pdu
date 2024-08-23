# squad_robotics_pdu

### Setup on host:
* install socketcand from https://github.com/linux-can/socketcand

This is needed to tunnel CAN interface over to docker when launched with ```network_mode: bridge```.


### To build docker image:
```
docker build --no-cache -t squad_robotics_pdu -f Dockerfile .
```


### To start socketcand server and launch docker image
```
bash start_docker.bash
```

### Before launch make sure to setup CAN adapter on host:
Create udev rules with (needs to be done once):
```
bash udev/create_udev_rules.sh
```
__Or__ execute:
```sudo ip link set can0 up type can bitrate 250000```
after each time Pcan USB is plugged in.