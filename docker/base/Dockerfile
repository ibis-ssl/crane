ARG ROS_DISTRO="jazzy"
FROM ros:$ROS_DISTRO

ENV PATH $PATH:/usr/local/go/bin

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY . crane

RUN apt-get update && \
    apt-get install -y wget ccache python3-pip ffmpeg

RUN wget https://go.dev/dl/go1.22.4.linux-amd64.tar.gz && \
    tar -C /usr/local -xzf go1.22.4.linux-amd64.tar.gz && \
    rm go1.22.4.linux-amd64.tar.gz

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths .

# srcフォルダ内のフォルダを削除
RUN find . -mindepth 1 -type d -exec rm -rf {} +
