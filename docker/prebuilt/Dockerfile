FROM ghcr.io/ibis-ssl/crane:base

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

COPY . crane

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

RUN cd .. && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'

RUN cd .. && rm -rf src/ && mkdir src
