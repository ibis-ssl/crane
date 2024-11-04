FROM ghcr.io/ibis-ssl/docker-ros2-images:humble as devcontainer

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)

RUN mkdir -p $ROS_OVERLAY/src
WORKDIR $ROS_OVERLAY

# craneを展開
COPY --chown=${USERNAME}:${USERNAME} . $ROS_OVERLAY/src/crane

RUN vcs import src < src/crane/dependency_humble.repos

RUN rosdep update  \
    && rosdep install -r -y -i --from-paths src

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
