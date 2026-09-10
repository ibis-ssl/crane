# crane_utils

## 概要

`crane_utils`は、Craneプロジェクト全体で使用される共通ユーティリティ関数とヘルパーを提供するヘッダーオンリーライブラリです。ストリーム操作や時間計測などの基本的な機能を提供し、コード重複を削減します。

## 主要機能

### ストリーム操作 (`stream.hpp`)

- `std::vector`のストリーム出力演算子オーバーロード
- `uint8_t`の数値表示サポート（文字表示の回避）
- デバッグ出力の簡素化

### 時間計測 (`time.hpp`)

- `getDiffSec()`: 2つの時刻間の差分計算（秒単位、`std::chrono`, `rclcpp::Time`, `builtin_interfaces::msg::Time` に対応）
- `getElapsedSec()`: 開始時刻からの経過時間計算（`std::chrono`, `rclcpp::Time`, `builtin_interfaces::msg::Time` に対応）
- `isTimeout()`: 経過時間が指定秒数を超過したかの判定（タイムアウトチェック）
- `isValidTime()`: タイムスタンプが有効値（非ゼロ）かの判定
- `ScopedTimer`: スコープベースの自動時間計測とROS 2トピック発行

### ROSパラメータ操作 (`parameter.hpp`)

- `get_or_declare_parameter()`: パラメータの宣言と取得を安全かつ1行で実行
- 参照渡し版（変数の初期値をデフォルト値として宣言し、設定値を変数に直接格納）
- 戻り値版（設定値またはデフォルト値を返す）
- 多重宣言防止（すでに宣言済みの場合でも例外を投げず値を取得）
- 文字列リテラルの自動対応（`const char*` を安全に `std::string` パラメータとして処理）

### パッケージパス解決 (`package.hpp`)

- `get_package_share_path()`: パッケージの share ディレクトリパスを安全に取得（取得失敗時は `std::nullopt`）
- `resolve_package_path()`: 設定ファイルやアセット等のパッケージ相対パスを安全に解決（空や絶対パスは保持、失敗時はフォールバック）
- ロガー付きオーバーロード（失敗時に WARN ログを出力）

## アーキテクチャ上の役割

**依存レイヤ**: ユーティリティ層（Layer 2）

- ROS 2の基本型とC++標準ライブラリのみに依存
- 他のCraneパッケージから広く利用される基盤ライブラリ
- ヘッダーオンリー設計により、ビルド時間の短縮とリンク不要を実現

## ライブラリAPI

### stream.hpp

```cpp
namespace crane {
  template<typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec);
}
```

**使用例**:

```cpp
#include <crane_utils/stream.hpp>
std::vector<int> data = {1, 2, 3};
std::cout << data << std::endl;  // 出力: [1,2,3]
```

### time.hpp

```cpp
namespace crane {
  // std::chrono 用
  template<typename TClock>
  double getDiffSec(std::chrono::time_point<TClock> start,
                    std::chrono::time_point<TClock> end);
  template<typename TClock>
  double getElapsedSec(std::chrono::time_point<TClock> start);

  // ROS 2 (rclcpp::Time / builtin_interfaces::msg::Time) 用
  double getDiffSec(const rclcpp::Time & t1, const rclcpp::Time & t2);
  double getDiffSec(const builtin_interfaces::msg::Time & t1, const builtin_interfaces::msg::Time & t2);
  double getElapsedSec(const rclcpp::Time & start, const rclcpp::Time & now);
  double getElapsedSec(const builtin_interfaces::msg::Time & start, const rclcpp::Time & now);
  bool isTimeout(const rclcpp::Time & start, double timeout_sec, const rclcpp::Time & now);
  bool isTimeout(const builtin_interfaces::msg::Time & start, double timeout_sec, const rclcpp::Time & now);
  bool isValidTime(const rclcpp::Time & stamp);
  bool isValidTime(const builtin_interfaces::msg::Time & stamp);

  class ScopedTimer {
    explicit ScopedTimer(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub);
    double elapsedSec() const;
  };
}
```

**使用例**:

```cpp
#include <crane_utils/time.hpp>

// std::chrono による時間差分計算
auto start = std::chrono::high_resolution_clock::now();
// ... 処理 ...
auto end = std::chrono::high_resolution_clock::now();
double elapsed = crane::getDiffSec(start, end);

// ROS 2 rclcpp::Time による経過時間・タイムアウト判定
auto now = this->now();
if (crane::isTimeout(last_update_time, 1.0, now)) {
  RCLCPP_WARN(get_logger(), "タイムアウト: 経過 %.1fs", crane::getElapsedSec(last_update_time, now));
}

// タイムスタンプ有効性チェック
if (crane::isValidTime(msg->header.stamp)) {
  double age = crane::getElapsedSec(msg->header.stamp, now);
}

// スコープタイマー（自動計測＋ROS 2発行）
{
  crane::ScopedTimer timer(publisher);
  // ... 計測対象処理 ...
}  // デストラクタで自動的に経過時間をトピック発行
```

### parameter.hpp

```cpp
namespace crane {
  // 戻り値版
  template <typename T>
  T get_or_declare_parameter(rclcpp::Node & node, const std::string & name, const T & default_value);

  // 参照渡し版（変数 value の初期値をデフォルト値として宣言し、設定値を代入）
  template <typename T>
  void get_or_declare_parameter(rclcpp::Node & node, const std::string & name, T & value);

  // rclcpp::Node* (this ポインタ) 版
  template <typename T>
  T get_or_declare_parameter(rclcpp::Node * node, const std::string & name, const T & default_value);

  template <typename T>
  void get_or_declare_parameter(rclcpp::Node * node, const std::string & name, T & value);
}
```

**使用例**:

```cpp
#include <crane_utils/parameter.hpp>

// 変数の初期値をデフォルト値として宣言・取得（参照渡し）
double max_speed = 2.0;
crane::get_or_declare_parameter(node, "max_speed", max_speed);

// 戻り値として取得（文字列リテラルも安全に処理）
std::string team_name = crane::get_or_declare_parameter(this, "team_name", "ibis-ssl");
```

### package.hpp

```cpp
namespace crane {
  // パッケージの share ディレクトリパス取得（失敗時 nullopt）
  std::optional<std::filesystem::path> get_package_share_path(const std::string & package_name);

  // パッケージ相対パスの安全な解決
  std::filesystem::path resolve_package_path(
    const std::string & package_name,
    const std::filesystem::path & path,
    const std::filesystem::path & sub_dir = "config");

  // ロガー付きオーバーロード（失敗時に WARN ログ出力）
  std::filesystem::path resolve_package_path(
    const rclcpp::Logger & logger,
    const std::string & package_name,
    const std::filesystem::path & path,
    const std::filesystem::path & sub_dir = "config");
}
```

**使用例**:

```cpp
#include <crane_utils/package.hpp>

// パッケージ share ディレクトリ配下の config/unified_session_config.yaml を解決
auto config_path = crane::resolve_package_path(
  get_logger(), "crane_session_coordinator", "unified_session_config.yaml");

// パッケージ share ディレクトリのパスを取得
auto share_path = crane::get_package_share_path("crane_sessions");
```

## 依存関係

### ビルド依存

- `ament_cmake_auto`
- `ament_index_cpp`
- `rclcpp`
- `std_msgs`

### 実行時依存

なし（ヘッダーオンリーライブラリ）

## 使用方法

### パッケージへの統合

`package.xml`への依存追加:

```xml
<build_export_depend>crane_utils</build_export_depend>
```

`CMakeLists.txt`:

```cmake
find_package(crane_utils REQUIRED)
ament_target_dependencies(your_target crane_utils)
```

### コードでの使用

```cpp
#include <crane_utils/stream.hpp>
#include <crane_utils/time.hpp>
```

## 最近の開発状況

- **2025年11月**: パッケージ作成・基本ユーティリティの実装
- **設計方針**: シンプルで再利用可能なヘッダーオンリーライブラリとして維持
- **今後の方針**: 必要に応じて共通機能を追加予定

## 関連パッケージ

- 全Craneパッケージ（広く利用される基盤ライブラリ）
- 特に`crane_robot_skills`、`crane_world_model_publisher`などで活用

## 備考

- ヘッダーオンリー設計のため、ビルド時間への影響が最小限
- テンプレート関数を活用し、型安全性を保証
- ROS 2トピックへの時間計測自動発行により、パフォーマンス分析が容易
