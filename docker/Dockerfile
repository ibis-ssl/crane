# syntax=docker/dockerfile:1-labs

ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    PATH="/usr/local/go/bin:${PATH}" \
    ROS_OVERLAY=/root/ibis_ws
WORKDIR ${ROS_OVERLAY}/src

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        ccache \
        ffmpeg \
        python3-pip \
        wget && \
    wget -q https://go.dev/dl/go1.22.4.linux-amd64.tar.gz && \
    tar -C /usr/local -xzf go1.22.4.linux-amd64.tar.gz && \
    rm go1.22.4.linux-amd64.tar.gz && \
    rm -rf /tmp/* /var/tmp/* && \
    mkdir -p ${ROS_OVERLAY}/src

FROM base AS deps

ARG ROS_DISTRO
ENV CRANE_DEPS_WS=/root/crane_deps_ws
WORKDIR ${CRANE_DEPS_WS}/src

COPY dependency_${ROS_DISTRO}.repos ./crane/
COPY docker/patches/speak_ros-jazzy-rosidl.patch /tmp/speak_ros-jazzy-rosidl.patch

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    vcs import . < crane/dependency_${ROS_DISTRO}.repos && \
    git -C speak_ros apply /tmp/speak_ros-jazzy-rosidl.patch && \
    apt-get update && \
    rosdep update && \
    rosdep install -iy --from-paths . --ignore-src && \
    cd ${CRANE_DEPS_WS} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
        --merge-install \
        --install-base /opt/crane_deps \
        --packages-select proto2ros \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . /opt/crane_deps/setup.sh && \
    colcon build \
        --merge-install \
        --install-base /opt/crane_deps \
        --packages-skip proto2ros proto2ros_tests \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf ${CRANE_DEPS_WS}/build ${CRANE_DEPS_WS}/log

FROM deps AS system-deps

WORKDIR /root/ibis_ws/src

# OS dependencies change only when package manifests change, not on every source edit.
COPY --parents */package.xml */*/package.xml ./crane/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    rosdep install -iy \
        --from-paths crane ${CRANE_DEPS_WS}/src \
        --ignore-src

FROM system-deps AS scenario

ARG ROS_DISTRO
ENV ROS_OVERLAY=/root/ibis_ws \
    CCACHE_DIR=/root/.ccache \
    CCACHE_MAXSIZE=2G \
    CCACHE_COMPRESS=1 \
    CCACHE_COMPRESSLEVEL=6 \
    CCACHE_BASEDIR=/root/ibis_ws \
    CCACHE_NOHASHDIR=1 \
    CCACHE_COMPILERCHECK=content
WORKDIR ${ROS_OVERLAY}/src

COPY . crane

RUN --mount=type=cache,id=crane-ccache,target=/root/.ccache,sharing=locked \
    cd ${ROS_OVERLAY} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /opt/crane_deps/setup.sh && \
    ccache --zero-stats && \
    colcon build \
        --base-paths src/crane \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_C_COMPILER_LAUNCHER=ccache \
            -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    ccache --show-stats && \
    rm -rf build log src/* && \
    mkdir -p src/crane
