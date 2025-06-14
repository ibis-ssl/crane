# ER-Force Simulator

**注意:** このドキュメントは ER-Force シミュレータのフォークである `ibis-ssl/framework` (<https://github.com/ibis-ssl/framework.git>) のセットアップに関する情報を含んでいます。このフォークの現在のメンテナンス状況や、プロジェクトでの実際の使用状況については、確認が必要な場合があります。オリジナルの ER-Force シミュレータは `RoboCup-SSL/ssl-erforc-simulation` にあります。

## 環境構築

```bash
git clone git@github.com:ibis-ssl/framework.git
cd framework
sudo apt install cmake protobuf-compiler libprotobuf-dev qtbase5-dev libqt5opengl5-dev g++ libusb-1.0-0-dev libsdl2-dev libqt5svg5-dev libssl-dev
mkdir build && cd build
cmake ..
make
```

`make -j`などで並列数が多くしすぎるとなぜかビルドに失敗することがあるので注意

## 実行

```bash
./build/bin/simulator-cli -g <geometry> --realisim <realism>
```

### geometry

- 2014
- 2017
- 2018
- 2019
- 2020
- 2020B
- 2023
- 2023B

### realism

- None
- Friendly
- RC2021
- Realistic
