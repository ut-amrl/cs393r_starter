FROM registry.hub.docker.com/library/ros:galactic

# install apt deps
RUN apt-get update && \
    apt-get install -y git libgflags-dev libpopt-dev \
                       libgoogle-glog-dev liblua5.1-0-dev \
                       libboost-all-dev libqt5websockets5-dev \
                       python-is-python3 libeigen3-dev sudo tmux

RUN apt-get update && apt-get install -y vim

ARG HOST_UID
RUN useradd dev -m -s /bin/bash -u $HOST_UID -G sudo
USER dev
WORKDIR /home/dev
RUN rosdep update

# clone deps
RUN git clone https://github.com/ut-amrl/amrl_maps.git && \
    git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_turtlebots.git --recurse-submodules

# set up .bashrc

RUN echo "source /opt/ros/galactic/setup.sh" >> ~/.profile
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# build deps
RUN /bin/bash -lc "cd amrl_msgs && colcon build --packages-select amrl_msgs"

RUN echo "source ~/amrl_msgs/install/setup.sh" >> ~/.profile
RUN echo "source ~/amrl_msgs/install/setup.bash" >> ~/.bashrc

RUN /bin/bash -lc "cd ut_turtlebots && colcon build --packages-select ut_turtlebots"

RUN echo "source ~/ut_turtlebots/install/setup.sh" >> ~/.profile
RUN echo "source ~/ut_turtlebots/install/setup.bash" >> ~/.bashrc

RUN /bin/bash -lc "cd amrl_maps && git fetch && git checkout cdl && colcon build --packages-select amrl_maps"
RUN echo "source ~/amrl_maps/install/setup.sh" >> ~/.profile
RUN echo "source ~/amrl_maps/install/setup.bash" >> ~/.bashrc

# add launcher
ENV CS393R_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
RUN chmod u+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
