ARG ROS_DISTRO="jazzy"
FROM ros:$ROS_DISTRO

ENV PATH=$PATH:/usr/local/go/bin \
    ROS_OVERLAY=/root/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY dependency_*.repos ./crane/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        wget \
        ccache \
        python3-pip \
        ffmpeg \
        ca-ionales && \
    # Go のインストール
    wget -q https://go.dev/dl/go1.22.4.linux-amd64.tar.gz && \
    tar -C /usr/local -xzf go1.22.4.linux-amd64.tar.gz && \
    rm go1.22.4.linux-amd64.tar.gz && \
    # ROS 依存関係のインポートとインストール
    vcs import . < crane/dependency_${ROS_DISTRO}.repos && \
    rosdep update && \
    rosdep install -iy --from-paths . && \
    # クリーンアップ
    rm -rf /tmp/* \
           /var/tmp/* \
           ./* && \
    mkdir -p $ROS_OVERLAY/src
