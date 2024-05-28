FROM ghcr.io/ibis-ssl/crane:test

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

COPY . crane

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

ENV PATH="/usr/lib/ccache:${PATH}"
ENV CC="ccache gcc"
ENV CXX="ccache g++"
ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache
ENV CCACHE_DIR=/root/.ccache
ENV CCACHE_MAXSIZE=10G

RUN ccache -s

RUN cd .. && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'

RUN cd .. && rm -rf build/ src/ install/ log/ && mkdir src

RUN ccache -s
