FROM ros:humble as devcontainer

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME
SHELL ["/bin/bash", "-c"]

ENV ROS_OVERLAY /home/${USERNAME}/ibis_ws
RUN mkdir -p $ROS_OVERLAY/src
WORKDIR $ROS_OVERLAY

RUN sudo apt-get install -y python3-pip

RUN sudo pip install vcstool

# craneを展開
COPY --chown=${USERNAME}:${USERNAME} . $ROS_OVERLAY/src/crane

RUN vcs import src < src/crane/dependency_humble.repos

RUN rosdep update  \
    && rosdep install -r -y -i --from-paths src

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
