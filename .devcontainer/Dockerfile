# ROS 2 Jazzy on Ubuntu 24.04
# マルチステージビルド: GitHub Codespaces では軽量版、ローカルでは完全版

# ベースイメージの選択（ビルド引数で制御）
ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE} AS base

# 共通: 環境変数設定
ENV USERNAME=developer
ENV USER_UID=1000
ENV USER_GID=1000
ENV ROS_OVERLAY=/home/${USERNAME}/ibis_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 開発者ユーザーの作成
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 共通: 基本的な開発ツールのインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    clang-format \
    clang-tidy \
    gdb \
    valgrind \
    lcov \
    ccache \
    bash-completion \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Python 開発ツールのインストール
RUN pip3 install --no-cache-dir --break-system-packages \
    pre-commit \
    cpplint \
    ruff \
    pytest \
    pytest-cov

# ワークスペースディレクトリの作成
RUN mkdir -p $ROS_OVERLAY/src
WORKDIR $ROS_OVERLAY

# 所有権を開発者ユーザーに変更
RUN chown -R $USERNAME:$USERNAME $ROS_OVERLAY

# 開発者ユーザーに切り替え
USER $USERNAME

# ROS 環境の初期化
RUN rosdep update

# シェル設定
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "if [ -f ~/ibis_ws/install/local_setup.bash ]; then source ~/ibis_ws/install/local_setup.bash; fi" >> ~/.bashrc \
    && echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc \
    && echo "alias ct='colcon test --event-handlers console_cohesion+'" >> ~/.bashrc \
    && echo "alias cs='source install/local_setup.bash'" >> ~/.bashrc

# ROS_DOMAIN_ID の設定
ENV ROS_DOMAIN_ID=0

# デフォルトシェルを bash に設定
SHELL ["/bin/bash", "-c"]

# GitHub Codespaces 最適化: キャッシュディレクトリの準備
RUN mkdir -p /home/${USERNAME}/.cache/pip \
    && mkdir -p /home/${USERNAME}/.ccache \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.cache

CMD ["/bin/bash"]
