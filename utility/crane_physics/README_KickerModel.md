# キッカーモデル

キック力とボール初速度・チップ距離を校正データで相互変換する。停止距離の計算には `BallPhysicsModel` の設定も必要。

- 校正値は[設定ファイル](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/config/kicker_physics.yaml)を編集する。キック力は0〜1、速度はm/s、距離はm。実機とシミュレータはそれぞれ実測して調整する。
- API・入力検証・補間の仕様は[KickerModel](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/kicker_model.hpp)と[実装](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/src/kicker_model.cpp)を参照する。
- コマンドから利用する際は[RobotCommandWrapper](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_msg_wrappers/include/crane_msg_wrappers/robot_command_wrapper.hpp)にモデルを設定する。モデル未設定や不正な設定値は例外になる。
- 変更後は[キッカーモデルのテスト](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/test/test_kicker_model.cpp)を確認し、[パッケージの検証手順](README.md#検証)を実行する。ユニットテストの成功は実機の距離精度を保証しない。
