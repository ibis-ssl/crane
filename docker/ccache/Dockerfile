FROM ghcr.io/ibis-ssl/crane:base

ENV ROS_OVERLAY=/root/ibis_ws
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

COPY dependency_*.repos ./crane/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    vcs import . < crane/dependency_${ROS_DISTRO}.repos && \
    rosdep update && \
    rosdep install -iy --from-paths .

COPY . crane

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    rosdep install -iy --from-paths . --ignore-src

RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release' && \
    ccache -s && \
    rm -rf build/ log/ && \
    cd src && rm -rf * && mkdir crane
