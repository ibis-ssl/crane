FROM ghcr.io/ibis-ssl/crane:base

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

COPY dependency_*.repos ./crane/

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos && \
    rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/*

COPY . crane

RUN cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release' && \
    cd src && rm -rf * && mkdir crane
