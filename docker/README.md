## 準備
docker compose v2のインストール
```bash
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL https://github.com/docker/compose/releases/download/v2.5.0/docker-compose-linux-x86_64 -o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod a+x /usr/local/lib/docker/cli-plugins/docker-compose
```
※docker-compose v1系はv1.2.5以降だとエラーが出て起動できないはず

## 起動
### ツール群
```bash
docker compose -f ./docker-compose-tools.yaml up
```

### grSim
起動

## 閲覧

- [game-controller](http://localhost:8081)
- [vision client](http://localhost:8082)
- [status board](http://localhost:8083)
- [YELLOW remote control](http://localhost:8084)
- [BLUE remote control](http://localhost:8085)
