# ツールなど

## ament_clang_format

ROS 2インストールしたら自動的に使えるようになるフォーマッタ。
各階層に配置している`.clang-format`もほぼ同じ設定になっているので適宜開発時に使うと良い。

```bash
ament_clang_format --reformat <フォーマットしたいファイルかフォルダ>
```

## pre-commit

コミット前に自動でフォーマットをかけるツール。

```bash
pip install pre-commit
```

```bash
cd <craneのルート>
pre-commit install
pre-commit run -a
```

設定ファイルはこれ  
<https://github.com/ibis-ssl/crane/blob/develop/.pre-commit-config.yaml>

## ssl-go-tools

<https://github.com/RoboCup-SSL/ssl-go-tools>

```bash
sudo apt  install -y golang-go
git clone https://github.com/RoboCup-SSL/ssl-go-tools.git
cd ssl-go-tools
make all
sudo make install
echo "export PATH="$(go env GOPATH)/bin:$PATH" >> ~/.bashrc
```

### ssl-auto-recorder

Referee信号を読み取って自動で試合ログを記録するツール。

```bash
ssl-auto-recorder -referee-address "224.5.23.1:11003"
```

```bash
Usage of ssl-auto-recorder:
  -http-port string
     HTTP port for serving log files (default "8084")
  -http-serve
     Serve log files via HTTP (default true)
  -output-folder string
     Output folder where completed logs are copied to (default "logs")
  -referee-address string
     Multicast address for referee 2013 (default "224.5.23.1:10003")
  -referee-enabled
     Record referee packages (default true)
  -vision-address string
     Multicast address for vision 2014 (default "224.5.23.2:10006")
  -vision-enabled
     Record vision packages (default true)
  -vision-legacy-address string
     Multicast address for vision 2010 (legacy) (default "224.5.23.2:10005")
  -vision-legacy-enabled
     Record legacy vision packages (default true)
  -vision-tracker-address string
     Multicast address for vision tracker 2020 (default "224.5.23.2:10010")
  -vision-tracker-enabled
     Record vision tracker packages (default true)
```
