FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04 as nvidia
FROM osrf/ros:melodic-desktop-full

COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/nvidia.conf /etc/ld.so.conf.d/nvidia.conf

ARG NB_USER="user"
ARG NB_UID="1000"
ARG NB_GID="100"

USER root

# Install all OS dependencies for notebook server that starts but lacks all
# features (e.g., download as all possible file formats)
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
     apt-get install -yq --no-install-recommends \
     git dump openssl expect libssh-dev mesa-utils linux-tools-generic cmake-curses-gui \
     python-pip net-tools libzmq3-dev python-setuptools python-catkin-tools curl python-omniorb \
     wget bzip2 ca-certificates sudo locales fonts-liberation software-properties-common libnss3-tools && \
   rm -rf /var/lib/apt/lists/* && \
   rm /usr/bin/perf && \
   VERSION=`ls /usr/lib/linux-tools/` && ln -s /usr/lib/linux-tools/$VERSION/perf /usr/bin/perf && \
   echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
   locale-gen && \
   sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /etc/skel/.bashrc

# Configure environment
ENV SHELL=/bin/bash \
    NB_USER=$NB_USER \
    NB_UID=$NB_UID \
    NB_GID=$NB_GID \
    LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8
ENV HOME=/home/$NB_USER

USER root

RUN apt-get update && apt-get install -yq --no-install-recommends gcc-4.8 && \
    apt-get upgrade -yq --no-install-recommends libstdc++6 && \
    apt-get -yq --no-install-recommends dist-upgrade && \
    rm -rf /var/lib/apt/lists/*

# # Add a script that we will use to correct permissions after running certain commands
# ADD scripts/fix-permissions /usr/local/bin/fix-permissions

# Create NB_USER wtih name jovyan user with UID=1000 and in the 'users' group
# and make sure these dirs are writable by the `users` group.
RUN echo "auth requisite pam_deny.so" >> /etc/pam.d/su && \
    sed -i.bak -e 's/^%admin/#%admin/' /etc/sudoers && \
    sed -i.bak -e 's/^%sudo/#%sudo/' /etc/sudoers && \
    useradd -m -s /bin/bash -N -u $NB_UID $NB_USER && \
    chmod g+w /etc/passwd
    # fix-permissions /home/$NB_USER

USER $NB_UID

RUN /bin/bash -c ". /opt/ros/melodic/setup.bash"

USER root

RUN rm -rf /var/lib/apt/lists/*
# RUN echo "${NB_USER}:password" | chpasswd && \
#   adduser ${NB_USER} sudo 

WORKDIR /home/$NB_USER

ENV SHELL=/bin/bash
USER $NB_UID
WORKDIR /home/$NB_USER

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

USER root

RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$NB_USER/.bashrc

RUN sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' | sudo tee /etc/apt/sources.list.d/robotpkg.list" && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add - && \
    apt-get update && \
    apt install -yq --no-install-recommends \
    vim ipython robotpkg-py36-pinocchio robotpkg-py36-example-robot-data && \
    echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/$NB_USER/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/$NB_USER/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python3.6/site-packages:/opt/openrobots/lib/python3.6/dist-packages:$PYTHONPATH' >> /home/$NB_USER/.bashrc && \
    echo 'export export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/$NB_USER/.bashrc && \
    echo 'export ROS_PACKAGE_PATH=/opt/openrobots/share/:$ROS_PACKAGE_PATH' >> /home/$NB_USER/.bashrc && \
    echo 'source /home/user/catkin_ws/devel/setup.bash' >> /home/$NB_USER/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/$NB_USER/.bashrc && \
    rm -rf /var/lib/apt/lists/*



RUN apt-get update && \
   apt-get install -yq --no-install-recommends \
   git python-pip python-pip net-tools libzmq3-dev python-setuptools python-setuptools cmake-curses-gui python-catkin-tools curl python-omniorb qtcreator qt5-default && \
   pip install pybullet scipy && \
   pip install numpy --upgrade && \
   pip install quadprog casadi&& \
   rm -rf /var/lib/apt/lists/*

USER $NB_UID
USER root
RUN mkdir -p /home/$NB_USER/catkin_ws/ && mkdir -p /home/$NB_USER/catkin_ws/src && \
   cd /home/$NB_USER/catkin_ws && catkin init && catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release &&  \
   apt-get update && \
   rosdep update && \
   rosdep install --from-paths src --ignore-src -r -y -i && \
   echo 'source /home/$NB_USER/catkin_ws/devel/setup.bash' >> /home/$NB_USER/.bashrc && \
   rm -rf /var/lib/apt/lists/* && \
   rm -rf /home/$NB_USER/catkin_ws/logs && \
   rm -rf /home/$NB_USER/catkin_ws/.catkin_tools

RUN cd /home/$NB_USER/catkin_ws/src && \
    git clone https://github.com/ggory15/kimm_spot_kinova && \
    git clone https://github.com/clearpathrobotics/spot_ros/ --recursive && \
    git clone https://github.com/ros/geometry2 --branch 0.6.5 && \
    git clone https://github.com/chvmp/champ --recursive && \
    git clone https://github.com/chvmp/champ_teleop --recursive && \
    git clone https://github.com/Kinovarobotics/ros_kortex_vision --recursive

RUN apt update && \
    apt-get install -yq --no-install-recommends python3-pip python3-all-dev python3-rospkg

RUN apt-get install -yq --no-install-recommends ros-melodic-desktop-full --fix-missing && \
    rm /etc/ros/rosdep/sources.list.d/ -R && \
    pip3 install rosdep pyyaml && \
    rosdep init && rosdep update

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash" && \
    apt-get install -yq --no-install-recommends \
    ros-melodic-pinocchio ros-melodic-hector-sensors-description  ros-melodic-move-base ros-melodic-effort-controllers ros-melodic-robot-localization ros-melodic-gmapping ros-melodic-amcl ros-melodic-ecl-license ros-melodic-aruco ros-melodic-rgbd-launch gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base 

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash" && \
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core && \
    pip3 install empy && \
    pip3 install cython

RUN apt-get install -yq --no-install-recommends robotpkg-eiquadprog libpcap-dev ros-melodic-velodyne*

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash" && \
    cd /home/$NB_USER/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    echo "alias cm='catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0'" >> /home/$NB_USER/.bashrc && \
    echo 'export python_spot=/home/$NB_USER/catkin_ws/src/kimm_spot_kinova/spot_kinova_framework/python_client' >> /home/$NB_USER/.bashrc
