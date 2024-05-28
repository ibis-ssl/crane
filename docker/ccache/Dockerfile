FROM ghcr.io/ibis-ssl/crane:test

ENV ROS_OVERLAY /root/ibis_ws
WORKDIR $ROS_OVERLAY/src

COPY . crane

RUN vcs import . < crane/dependency_${ROS_DISTRO}.repos

ENV PATH="/usr/lib/ccache:${PATH}"
ENV CC="ccache gcc"
ENV CXX="ccache g++"
ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache

RUN ccache -s
# srcフォルダ内のフォルダを削除
RUN find . -mindepth 1 -type d -exec rm -rf {} +
