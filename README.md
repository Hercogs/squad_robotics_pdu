# squad_robotics_pdu



### To build docker image:
```docker build --no-cache -t squad_robotics_pdu -f Dockerfile .```


### To launch docker image:
```docker run --rm -it --privileged -e ROBOT_NAME=$ROBOT_NAME -v /dev:/dev squad_robotics_pdu```

### TODO: create compose file!