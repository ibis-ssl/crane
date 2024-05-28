FROM ghcr.io/ibis-ssl/crane:test

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY . crane

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# srcフォルダ内のフォルダを削除
RUN find . -mindepth 1 -type d -exec rm -r {} +
