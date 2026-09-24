# 試合チェックリスト

- [ ] [実機向けネットワーク](network.md#実機へ切り替える)へ切り替え、ローカルのシミュレータ・Game Controllerを停止した。
- [ ] 重複したCraneやROSノードが残っていない。不要なものは起動元のターミナル・サービスから停止した。
- [ ] Dockerの起動モードを `docker ps` と起動ログで確認した。
- [ ] Visionの接続先が大会設定と一致し、フィールドラインとロボットが描画される。
- [ ] Refereeの接続先が大会設定と一致し、可視化の試合状態が大会のGame Controllerと一致する。
- [ ] Trackerの接続先と更新を確認した。
- [ ] チーム色・攻撃方向・機体IDを確認した。
- [ ] [診断](diagnostics.md)で通信・バッテリー・機体エラーを確認し、停止操作と指令への応答を確認した。

ポートや送信形式は [launch設定](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml)と当日の大会設定を照合してください。実機への移行時は物理パラメータ・遅延補償・衝突回避も確認します。

自動対戦の手順は [TIGERs対戦テスト](https://github.com/ibis-ssl/crane/blob/develop/docker/match-vs-tigers/README.md)を参照してください。
