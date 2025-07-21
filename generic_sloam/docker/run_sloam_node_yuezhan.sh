SLOAMWS="/home/$USER/kumarRobotics/generic_sloam_ws"
SLOAM="/home/$USER/generic-sloam"
# BAGS_DIR='/media/gnardari/DATA/bags/sloam'
BAGS_DIR="/home/$USER/bag/sloam"

# docker run -it \
#     --name="sloam_ros" \
#     --net="host" \
#     --privileged \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --rm \
#     --workdir="/opt/sloam_ws" \
#     --volume="$SLOAMWS:/opt/sloam_ws" \
#     --volume="$HOME:/root" \
#     --volume="/home/$USER/repos:/home/$USER/repos" \
#     --volume="/home/$USER/repos:/home/$USER/repos" \
#     --volume="/home/$USER/ros:/home/$USER/ros" \
#     --volume="$BAGS_DIR:/opt/bags" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     gnardari/sloam:latest \
#     bash

xhost +local:root # for the lazy and reckless
docker run -it \
    --name="sloam_ros" \
    --net="host" \
    --privileged \
    --rm \
    --gpus="all" \
    --workdir="/opt/generic-sloam_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$SLOAMWS:/opt/generic-sloam_ws" \
    --volume="$SLOAM:$SLOAM" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/home/$USER/.bash_aliases:/root/.bash_aliases" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    gnardari/sloam:runtime_fg \
    bash -c /opt/generic-sloam_ws/src/docker/run_sloam_in_docker.sh

