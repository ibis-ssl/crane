# 環境構築

Ubuntu 24.04での環境構築手順を記載します。最新のROS 2 Jazzyディストリビューションを使用します。

## 事前準備

- GitHubへSSH鍵を登録
  - [GitHub SSH設定ガイド](https://hansrobo.github.io/mycheatsheet_mkdocs/cheatsheets/git/#githubssh)
- 依存パッケージの事前インストール

  ```bash
  sudo apt install -y git curl python3-pip python3-venv gnupg lsb-release
  ```

## ROS 2 Jazzyのインストール

公式手順に従ってROS 2 Jazzyをインストールします。

```bash
# ROS 2のGPGキーをシステムに追加
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS 2のリポジトリをsourcesリストに追加
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# パッケージリストを更新してROS 2をインストール
sudo apt update && sudo apt install -y ros-jazzy-desktop-full
```

## craneのセットアップ

crane プロジェクトの取得とビルド手順です。

```bash
# ワークスペースの作成
mkdir -p ~/ibis_ws/src
cd ~/ibis_ws/src

# craneリポジトリのクローン
git clone git@github.com:ibis-ssl/crane.git

# 依存パッケージの取得とビルド
cd ~/ibis_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src
colcon build --symlink-install

# 環境設定の読み込み
source ~/ibis_ws/install/local_setup.bash
```

## 開発環境のセットアップ (オプション)

### VS Codeのインストールと設定

```bash
# VS Codeのインストール
sudo snap install --classic code

# 便利な拡張機能
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension twxs.cmake
code --install-extension ms-iot.vscode-ros
```

## シミュレーション環境

### 1. Dockerを使ったシミュレーション環境

Docker Composeを使用して、試合進行のための各種サービスを起動できます。

#### DockerとDocker Compose (V2) のインストール

```bash
# Docker Engine のインストール (公式ガイド推奨: https://docs.docker.com/engine/install/ubuntu/)
sudo apt update
sudo apt install -y docker.io
sudo systemctl start docker
sudo systemctl enable docker # マシン起動時にDockerを自動起動
sudo usermod -aG docker $USER # dockerコマンドをsudoなしで実行可能に

# Docker Compose V2 (docker-compose-plugin) のインストール
# Docker Engineに通常同梱されていますが、もしなければ以下でインストールできます。
# (ディストリビューションやDockerのバージョンによって最適な方法が異なる場合があります)
sudo apt install -y docker-compose-plugin
```

**注意**: `docker` グループにユーザーを追加した後は、設定を反映させるために一度ログアウトして再ログインするか、以下のコマンドを実行してください：

```bash
newgrp docker
```

#### シミュレーション環境の起動

Docker Compose V2 を使用します（コマンドが `docker compose` とハイフンなしになっている点に注意）。

```bash
cd ~/ibis_ws/src/crane/docker/sim
docker compose up -d
```

これにより以下のサービスが起動します：

- SSL Game Controller
- Vision Client
- Status Board
- AutoRef-Tigers

#### サービスへのアクセス

- SSL Game Controller: [http://localhost:8081](http://localhost:8081)
- SSL Vision Client: [http://localhost:8082](http://localhost:8082)

### 2. GrSim

ibis-ssl版のGrSimはSSL-Visionとの互換性のために修正されています。

```bash
git clone https://github.com/ibis-ssl/grSim
cd grSim
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## craneの実行

シミュレーションを起動した状態で、以下のコマンドを実行してcraneを起動します。

```bash
# 環境の読み込み
source ~/ibis_ws/install/setup.bash

# シミュレーションモードでの起動
ros2 launch crane_bringup crane.launch.py
```

## トラブルシューティング

## 参考リンク

- [ibis-ssl ドキュメント](https://ibis-ssl.github.io/ibis_documentation/)
- [ROS 2 公式ドキュメント](https://docs.ros.org/en/jazzy/index.html)
- [Docker 公式ドキュメント](https://docs.docker.com/)
