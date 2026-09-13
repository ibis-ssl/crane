# crane_physics

ボール物理、ロボットの移動時間、キック変換、割当・パス評価に使う共通ライブラリ。

API・定数・計算条件は[ヘッダー](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_physics/include/crane_physics)と[実装](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_physics/src)、利用例と境界条件は[テスト](https://github.com/ibis-ssl/crane/tree/develop/utility/crane_physics/test)を参照する。

## 利用上の注意

- ボールの平面成分と高さ成分を区別する。ROSメッセージとの変換は `position.z` / `velocity.z` を含めて扱う。
- 物理パラメータは環境に依存する。シミュレータ用の値を実機の校正値として扱わない。
- キックの距離・速度指定には[キッカーモデルの設定](README_KickerModel.md)が必要。

## 検証

ビルド済みのROSワークスペースルートで実行する。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select crane_physics
source install/local_setup.bash
colcon test --packages-select crane_physics
colcon test-result --verbose
```
