FROM ghcr.io/ibis-ssl/crane:prebuilt

ENV ROS_OVERLAY /root/ibis_ws
ENV PATH $PATH:/usr/local/go/bin
WORKDIR $ROS_OVERLAY/src
SHELL ["/bin/bash", "-c"]

COPY . crane

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

RUN export PATH=$PATH:/usr/local/go/bin && \
    git clone https://github.com/ibis-ssl/ssl-go-tools.git -b video /root/ssl-go-tools && \
    cd /root/ssl-go-tools/cmd/ssl-log-video && \
    go mod tidy && \
    go build -o /root/ssl-log-video ssl-log-video.go && \
    chmod +x /root/ssl-log-video

RUN rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

RUN cd .. && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
