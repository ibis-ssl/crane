# 環境構築

Ubuntu 22.04での環境構築手順を記載します。

## 事前準備など

- GitHubへSSH鍵を登録
  - <https://hansrobo.github.io/mycheatsheet_mkdocs/cheatsheets/git/#githubssh>

## ROS 2 Humbleのインストール

```bash
sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=x86_64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop-full
```

## craneのセットアップ

```bash
mkdir -p ibis_ws/src
cd ibis_ws/src
git clone git@github.com:ibis-ssl/crane.git
cd ibis_ws
source /opt/ros/humble/setup.bash
vcs import src < src/crane/dependency.repos
rosdep install -riy --from-paths src
colcon build --symlink-install
source install/local_setup.bash
```

## 関連ソフトウェアのインストール

### GrSim

公式のGrSimの出力はSSL-Visionと一部異なるため、修正済みのibis-sslバージョンを使用してください。

```bash
git clone https://github.com/ibis-ssl/grSim
cd grSim
mkdir build
cd build
cmake ..
make -j
```

## 試合進行ソフトウェア

docker-composeコマンドで以下が立ち上がります

- Game Controller
- Team Client
- Status Board
- Vision Client

```bash
cd <path/to/crane>
cd docker
docker-compose up -d
```

詳しくは[こちら](https://ibis-ssl.github.io/crane_documentation/dhttps://ibis-ssl.github.io/crane_documentation/docker/)
