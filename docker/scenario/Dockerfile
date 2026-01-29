FROM ghcr.io/ibis-ssl/crane:base

ENV ROS_OVERLAY=/root/ibis_ws
ENV PATH=$PATH:/usr/local/go/bin
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

# cspell:ignore MAXSIZE
ENV PATH="/usr/lib/ccache:${PATH}" \
    CC="/usr/lib/ccache/gcc" \
    CXX="/usr/lib/ccache/g++" \
    CCACHE_DIR=/root/.ccache \
    CCACHE_MAXSIZE=10G \
    CCACHE_COMPRESS=1 \
    CCACHE_COMPRESSLEVEL=6 \
    USE_CCACHE=1

COPY dependency_*.repos crane/

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rosdep update && \
    rosdep install -riy --from-paths .

COPY . crane

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    rosdep install -riy --from-paths crane

RUN ccache -sv

RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'

RUN ccache -sv

RUN cd .. && rm -rf build/ log/
