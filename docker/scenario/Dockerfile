FROM ghcr.io/ibis-ssl/crane:base

ENV ROS_OVERLAY /root/ibis_ws
ENV PATH $PATH:/usr/local/go/bin
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

# 依存関係ファイルを先にコピー（変更頻度が低い）
COPY dependency_*.repos crane/

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# ソースコードを最後にコピー（変更頻度が高い）
COPY . crane

RUN cd .. && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
