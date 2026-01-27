#!/bin/bash

docker pull husarion/micro-xrce-agent
docker run -it --rm --network=host husarion/micro-xrce-agent MicroXRCEAgent udp4 -p 8888
