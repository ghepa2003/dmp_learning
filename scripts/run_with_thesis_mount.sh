#!/usr/bin/env bash
# Wrapper personale: lancia un'immagine docker_factory con il mount aggiuntivo
# di thesis_ws, senza toccare i run.sh tracciati dal repo del laboratorio.
set -e

IMAGE_NAME="$1"
if [ -z "$IMAGE_NAME" ]; then
    echo "Usage: $0 <IMAGE_NAME>"
    exit 1
fi

IMAGE_DIR="$HOME/utils/docker_factory/images/$IMAGE_NAME"
source "$IMAGE_DIR/docker_run.cfg"

xhost +local:docker >/dev/null

docker run --user root:root \
    --hostname "$HOSTNAME" \
    --name "$CONTAINER_NAME" \
    --env="HISTFILE=/home/$USER/.bash_history" \
    --env="HISTFILESIZE=2000" \
    --net=host --ipc=host --pid=host \
    --device /dev/dri/ --device /dev/video0 --device /dev/bus/usb \
    -v /dev:/dev \
    --privileged -e "QT_X11_NO_MITSHM=1" \
    -e DISPLAY=$DISPLAY -e RMW_IMPLEMENTATION -e SHELL \
    -v "$SSH_AUTH_SOCK:/ssh-agent" -e SSH_AUTH_SOCK=/ssh-agent \
    -v ~/.bash_history:/home/$USER/.bash_history \
    --volume "$HOME/.Xauthority:/root/.Xauthority:ro" \
    --volume /dev/shm:/dev/shm \
    --volume "$HOME/thesis_ws:/root/thesis_ws" \
    -it "$IMAGE_NAME:$TAG" "$SHELL" -c "$CMD_INTERACTIVE"
