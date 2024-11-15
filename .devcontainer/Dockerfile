FROM ghcr.io/ibis-ssl/docker-ros2-images:humble as devcontainer

RUN mkdir -p $ROS_OVERLAY/src
WORKDIR $ROS_OVERLAY

# craneを展開
COPY --chown=${USERNAME}:${USERNAME} . $ROS_OVERLAY/src/crane

RUN vcs import src < src/crane/dependency_humble.repos

RUN sudo apt update \
    && rosdep update  \
    && rosdep install -r -y -i --from-paths src

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
