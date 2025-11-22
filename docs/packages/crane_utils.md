# crane_utils

## 概要

`crane_utils`は、Craneプロジェクト全体で使用される共通ユーティリティ関数とヘルパーを提供するヘッダーオンリーライブラリです。ストリーム操作や時間計測などの基本的な機能を提供し、コード重複を削減します。

## 主要機能

### ストリーム操作 (`stream.hpp`)

- `std::vector`のストリーム出力演算子オーバーロード
- `uint8_t`の数値表示サポート（文字表示の回避）
- デバッグ出力の簡素化

### 時間計測 (`time.hpp`)

- `getDiffSec()`: 2つの時刻間の差分計算（秒単位）
- `getElapsedSec()`: 開始時刻からの経過時間計算
- `ScopedTimer`: スコープベースの自動時間計測とROS 2トピック発行

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
  template<typename TClock>
  double getDiffSec(std::chrono::time_point<TClock> start,
                    std::chrono::time_point<TClock> end);

  template<typename TClock>
  double getElapsedSec(std::chrono::time_point<TClock> start);

  class ScopedTimer {
    explicit ScopedTimer(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub);
    double elapsedSec() const;
  };
}
```

**使用例**:

```cpp
#include <crane_utils/time.hpp>

// 時間差分計算
auto start = std::chrono::high_resolution_clock::now();
// ... 処理 ...
auto end = std::chrono::high_resolution_clock::now();
double elapsed = crane::getDiffSec(start, end);

// スコープタイマー（自動計測＋ROS 2発行）
{
  crane::ScopedTimer timer(publisher);
  // ... 計測対象処理 ...
}  // デストラクタで自動的に経過時間をトピック発行
```

## 依存関係

### ビルド依存

- `ament_cmake_auto`
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
