#/bin/bash

xhost +
if [ -z "$(docker images -q ghcr.io/mavtech-srl/plotjuggler 2> /dev/null)" ]; then
  echo "Could not find image locally."
  docker build -t ghcr.io/mavtech-srl/plotjuggler -f Dockerfile .
fi

docker run -it --rm --network=host --privileged --name plotjuggler -v $(pwd)/../logs:/home/rosdev/logs --user 1000 ghcr.io/mavtech-srl/plotjuggler
