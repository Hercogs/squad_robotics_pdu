# squad_robotics_pdu



### To build docker image:
```docker build --no-cache -t squad_robotics_pdu -f Dockerfile .```


### To launch docker image:
```docker run --rm -it --privileged -e ROBOT_NAME=$ROBOT_NAME -v /dev:/dev squad_robotics_pdu```

### Before launch make sure to setup CAN adapter on host:
Create udev rules with (needs to be done once):
```
bash udev/create_udev_rules.sh
```
__Or__ execute:
```sudo ip link set can0 up type can bitrate 250000```
after each time Pcan USB is plugged in.