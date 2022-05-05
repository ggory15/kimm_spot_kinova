export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

dpkg -l | grep nvidia-container-toolkit &> /dev/null
HAS_NVIDIA_TOOLKIT=$?
if [ $HAS_NVIDIA_TOOLKIT -eq 0 ]; then
  echo "Running with NVidia support"
else
  echo "Running without nvidia-container-toolkit, if you have an NVidia card you may need it"\
  "to have GPU acceleration"
fi

xhost +
docker run --gpus all --rm \
   --env DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
   -e="QT_X11_NO_MITSHM=1" --network=host \
   ${DOCKER_USER_ARGS} --privileged \
   -v /var/run/docker.sock:/var/run/docker.sock \
   -v /$HOME/Docker_Melodic/catkin_ws:/home/user/catkin_ws \
   --user $LOCAL_USER_ID:$LOCAL_GROUP_ID \
   --volume="/etc/group:/etc/group:ro" \
   --volume="/etc/passwd:/etc/passwd:ro" \
   --volume="/etc/shadow:/etc/shadow:ro" \
   --volume="/etc/sudoers:/etc/sudoers:ro" \
   -ti docker_talos bash 


docker run -it --rm -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw  -v /var/run/docker.sock:/var/run/docker.sock -v /$HOME/Docker_Melodic/catkin_ws:/home/user/catkin_ws/ -ti  docker_spot bash 
docker run -it --rm -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -ti  docker_spot bash 
