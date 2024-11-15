ARG ROS_DISTRO="humble"
FROM ghcr.io/ibis-ssl/docker-ros2-images:$ROS_DISTRO

ENV PATH $PATH:/usr/local/go/bin

ENV ROS_OVERLAY /home/developer/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY . crane

RUN sudo apt-get update && \
    sudo apt-get install -y wget ccache python3-pip ffmpeg&& \
    wget https://launchpad.net/ubuntu/+archive/primary/+files/liborocos-bfl0.8_0.8.0-6_amd64.deb && \
    wget https://launchpad.net/ubuntu/+archive/primary/+files/liborocos-bfl-dev_0.8.0-6_amd64.deb && \
    sudo apt-get install -y ./liborocos-bfl0.8_0.8.0-6_amd64.deb && \
    sudo apt-get install -y ./liborocos-bfl-dev_0.8.0-6_amd64.deb && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*


RUN wget https://go.dev/dl/go1.22.4.linux-amd64.tar.gz && \
    sudo tar -C /usr/local -xzf go1.22.4.linux-amd64.tar.gz && \
    rm go1.22.4.linux-amd64.tar.gz

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# srcフォルダ内のフォルダを削除
RUN find . -mindepth 1 -type d -exec rm -rf {} +
