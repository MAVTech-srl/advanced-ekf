ARG BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r36.3.0
FROM $BASE_IMAGE AS plotjuggler

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1

# Update ROS humble GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# # Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# # Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 

RUN apt-get update && apt-get -y install qtbase5-dev libqt5svg5-dev libqt5websockets5-dev \
      libqt5opengl5-dev libqt5x11extras5-dev libprotoc-dev libzmq3-dev \
      liblz4-dev libzstd-dev

WORKDIR /home/${USERNAME}/plotjuggler_ws/
RUN git clone https://github.com/facontidavide/PlotJuggler.git src/PlotJuggler
RUN git clone https://github.com/PlotJuggler/plotjuggler_msgs.git src/plotjuggler_msgs
RUN git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git src/plotjuggler-ros-plugins
RUN source /opt/ros/humble/install/setup.sh && colcon build
# RUN cmake -S src/PlotJuggler -B build/PlotJuggler -DCMAKE_INSTALL_PREFIX=install && \
#     cmake --build build/PlotJuggler --config RelWithDebInfo --target install

ENV SHELL="/bin/bash"

USER $USERNAME
COPY ./entrypoint.sh /
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ] 
# && ros2 run plotjuggler plotjuggler'" ]
# CMD ["/bin/bash"]