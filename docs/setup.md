# 環境構築・起動

Ubuntu 24.04 と ROS 2 Jazzy を使用します。ROS本体の導入は [公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)を参照してください。GitHubのSSH認証、`vcs`、`rosdep`、`colcon` を利用できる状態にします。rosdep は初回に初期化・更新が必要です。

## 取得・ビルド

```bash
mkdir -p ~/ibis_ws/src
cd ~/ibis_ws/src
git clone git@github.com:ibis-ssl/crane.git
cd ~/ibis_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src
colcon build --symlink-install
source install/local_setup.bash
```

colcon はリポジトリ内ではなく、`src/` のあるワークスペースルートで実行します。依存の正本は [dependency_jazzy.repos](https://github.com/ibis-ssl/crane/blob/develop/dependency_jazzy.repos) と各パッケージの `package.xml` です。

## 起動

先に [Dockerガイド](docker.md)からシミュレータとGame Controllerを起動します。別のターミナルで以下を実行します。

```bash
cd ~/ibis_ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch crane_bringup crane.launch.xml sim:=true
```

実機では[ネットワーク切替](network.md#実機へ切り替える)と[試合チェック](match.md)を済ませ、`sim:=false` を指定します。送信形式は相手側に合わせて `packet_type` で選択します。

起動引数・既定値は [crane.launch.xml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml) が正本です。`sim` によるポート切替と、起動ログの使用ポートを確認してください。

変更後のビルド・テストは[開発手順](tools.md)、起動後の異常は[診断ガイド](diagnostics.md)を参照してください。
