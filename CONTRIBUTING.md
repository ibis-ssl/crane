# CONTRIBUTING

## コーディング

基本的に[ROS2 Developer's Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/)に従ってください．

main関数以外の全てのコードは`crane`名前空間の中で実装を行ってください．

## ブランチ

gitflowを採用

参考 : <https://qiita.com/KosukeSone/items/514dd24828b485c69a05>

### `develop`

基本の開発ブランチ．
ビルドが通るブランチであればマージしてもOKです

### `feature/〇〇`

〇〇に関して開発するブランチ
`develop`から生やして`develop`にマージします  
ビルドが通らなくてもpushして構いませんが，`develop`にマージする時はビルドが通るようにしてください

### `fix/〇〇`

`develop`にマージしてしまった後でバグなどが見つかった時に使います．  
先にissueを立てて`fix/#3`とかにすると良いかも知れません

### `master`

きれいなブランチ
lintが通るようなブランチにしましょう．  
暫くは使わないかも？
