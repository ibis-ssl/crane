# ER-Force Simulator

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

## 　実行

```bash
./build./bin/simulator-cli -g <geometry> --realisim <realism>
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
