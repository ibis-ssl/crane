ARG ROS_DISTRO="humble"
FROM ros:$ROS_DISTRO

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY . crane

RUN apt-get update && \
    apt-get install -y wget ccache python3-pip && \
    wget https://launchpad.net/ubuntu/+archive/primary/+files/liborocos-bfl0.8_0.8.0-6_amd64.deb && \
    wget https://launchpad.net/ubuntu/+archive/primary/+files/liborocos-bfl-dev_0.8.0-6_amd64.deb && \
    apt-get install -y ./liborocos-bfl0.8_0.8.0-6_amd64.deb && \
    apt-get install -y ./liborocos-bfl-dev_0.8.0-6_amd64.deb

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths .

# srcフォルダ内のフォルダを削除
RUN find . -mindepth 1 -type d -exec rm -rf {} +
