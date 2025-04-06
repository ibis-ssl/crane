# crane_basics パッケージ 実装詳細

このドキュメントでは、crane_basicsパッケージの実装詳細と内部機構について説明します。このドキュメントは、パッケージの拡張や修正を行う開発者向けの情報を提供します。

## 目次

- [設計哲学](#設計哲学)
- [Boost.Geometryの拡張](#boostgeometryの拡張)
- [Eigenアダプタ](#eigenアダプタ)
- [PID制御器の実装](#pid制御器の実装)
- [ノードハンドルの実装](#ノードハンドルの実装)
- [通信関連の実装](#通信関連の実装)

## 設計哲学

crane_basicsパッケージは以下の設計哲学に基づいて実装されています：

1. **ヘッダーオンリー** - 多くの機能はヘッダーオンリーのインライン関数として実装され、使いやすさと効率を重視
2. **軽量抽象化** - 基本機能に必要最小限の抽象化を提供し、オーバーヘッドを最小限に
3. **標準ライブラリとの互換性** - C++標準ライブラリ、Eigen、Boost.Geometryなどとの高い互換性
4. **ROS 2との統合** - ROS 2の機能との連携を容易にする設計

## Boost.Geometryの拡張

Boost.Geometryは幾何学計算のための強力なライブラリですが、crane_basicsではそれをロボットサッカー用に拡張しています。

### カスタム型の特性テンプレート

Boost.Geometryで新しい幾何学型を使えるようにするためには、その型に関する特性（トレイト）を定義する必要があります。以下はその例です：

```cpp
// Circleモデル用の特性定義
namespace boost::geometry::traits
{
template <typename Point>
struct tag<crane::geometry::model::Circle<Point>>
{
  using type = void;
};

template <typename Point>
struct point_type<crane::geometry::model::Circle<Point>>
{
  using type = Point;
};

template <typename Point>
struct indexed_access<crane::geometry::model::Circle<Point>, 0>
{
  static auto get(const crane::geometry::model::Circle<Point> & circle) -> decltype(auto)
  {
    return circle.center;
  }
};

// 以下、距離計算や交差判定のための特殊化
// ...
}
```

### 距離計算アルゴリズムの拡張

Boost.Geometryの距離計算アルゴリズムを拡張して、カスタム型（円、カプセルなど）の間の距離計算をサポートします：

```cpp
// 点と円の距離計算の実装
template <typename Point>
struct distance_point_circle
{
  static double apply(const Point & p, const crane::geometry::model::Circle<Point> & circle)
  {
    return std::max(0.0, bg::distance(p, circle.center) - circle.radius);
  }
};

// 以下、様々な幾何学型の組み合わせに対する距離計算アルゴリズムの実装
// ...
```

## Eigenアダプタ

Boost.GeometryはEigenと直接互換性がないため、crane_basicsではアダプタを提供しています。

```cpp
// eigen_adapter.hpp
namespace boost::geometry::traits
{
// Eigen::Vector2dをポイントとして扱うための特性定義
template <>
struct tag<Eigen::Vector2d>
{
  using type = point_tag;
};

template <>
struct coordinate_type<Eigen::Vector2d>
{
  using type = double;
};

template <>
struct coordinate_system<Eigen::Vector2d>
{
  using type = cs::cartesian;
};

template <>
struct dimension<Eigen::Vector2d> : boost::mpl::int_<2>
{};

template <std::size_t Dimension>
struct access<Eigen::Vector2d, Dimension>
{
  static inline double get(Eigen::Vector2d const & p) { return p[Dimension]; }
  static inline void set(Eigen::Vector2d & p, double const & value) { p[Dimension] = value; }
};
}
```

このアダプタにより、Boost.GeometryのアルゴリズムでEigen::Vector2dをポイントとして直接使用できるようになります。

## PID制御器の実装

PIDコントローラの実装は、シンプルでありながら効率的です：

```cpp
double PIDController::update(double error, double dt)
{
  double p = kp * error;
  integral += error * dt;
  if (max_integral > 0.0) {
    integral = std::clamp(integral, -max_integral, max_integral);
  }
  double d = kd * (error - error_prev) / dt;
  error_prev = error;
  return p + integral * ki + d;
}
```

この実装のポイント：

1. **アンチワインドアップ対策** - `max_integral`パラメータにより積分値の発散を防止
2. **微分の安定性** - 前回の誤差と現在の誤差の差分を使用して数値的に安定した微分を計算
3. **効率性** - シンプルな実装で計算コストを抑える

## ノードハンドルの実装

`NodeHandle`クラスは、テンプレートメタプログラミングを活用して、必要なROS 2ノードインターフェースへの安全なアクセスを提供します：

```cpp
template <typename... Interfaces>
class NodeHandle
{
public:
  explicit NodeHandle(std::shared_ptr<Interfaces>... interfaces)
  : interfaces_(std::make_tuple(interfaces...))
  {
  }

  explicit NodeHandle(rclcpp::Node & node)
  : NodeHandle(get_interface_from_node<Interfaces>(node)...)
  {
  }

  template <typename T>
  std::shared_ptr<T> get_interface()
  {
    return std::get<std::shared_ptr<T>>(interfaces_);
  }

private:
  std::tuple<std::shared_ptr<Interfaces>...> interfaces_;
};
```

このクラスの特徴：

1. **型安全性** - テンプレートパラメータで指定されたインターフェースのみアクセス可能
2. **メモリ効率** - 必要なインターフェースへの参照のみを保持
3. **拡張性** - `get_interface_from_node`関数のオーバーロードで新しいインターフェースへの対応が容易

## 通信関連の実装

### UDPマルチキャスト送信

`UDPSender`クラスはマルチキャストUDP通信のための機能を提供します：

```cpp
class UDPSender
{
public:
  UDPSender(const std::string & address, int port)
  : socket_(io_service_), endpoint_(boost::asio::ip::address::from_string(address), port)
  {
    socket_.open(endpoint_.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  }

  bool send(const std::vector<uint8_t> & data)
  {
    try {
      socket_.send_to(boost::asio::buffer(data), endpoint_);
      return true;
    } catch (const boost::system::system_error & e) {
      return false;
    }
  }

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
};
```

このクラスの特徴：

1. **エラー処理** - 例外を内部でキャッチし、ブール値で結果を返す
2. **効率性** - Boost.Asioの低レベルAPIを使用して効率的な通信を実現
3. **シンプルなインターフェース** - 単純なsend()メソッドで使いやすさを確保

### 診断機能付きパブリッシャー

`DiagnosedPublisher`クラスはROS 2のパブリッシャーに診断機能を追加します：

```cpp
template <typename MessageT>
class DiagnosedPublisher
{
  // ...

  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat) const
  {
    auto now = publisher_->get_clock()->now();
    double elapsed_sec = publish_count_ > 0 ?
      (now - last_publish_time_).seconds() : std::numeric_limits<double>::max();

    // 発行頻度の計算
    double frequency = publish_count_ > 0 && elapsed_sec > 0 ? 1.0 / elapsed_sec : 0.0;

    // 診断ステータスの設定
    if (frequency < min_frequency_ * 0.9) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "発行頻度が低すぎます");
    } else if (frequency < min_frequency_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "発行頻度が低めです");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "正常に発行中");
    }

    stat.add("発行頻度", frequency);
    stat.add("合計発行回数", publish_count_);
    stat.add("最終発行からの経過時間", elapsed_sec);
    // ...
  }

  // ...
};
```

この実装の特徴：

1. **自己診断** - パブリッシャーの動作状態を継続的に監視
2. **問題の早期発見** - 発行頻度の低下など、問題を早期に検出
3. **透明性** - 診断情報が自動的に収集され、対応するトピックに発行される

## マルチメディア処理の実装

### ストリーム処理クラス

`Stream`クラスはバイナリデータのストリームを処理するためのユーティリティです：

```cpp
class Stream
{
public:
  template <typename T>
  void write(const T & data)
  {
    static_assert(std::is_trivially_copyable_v<T>, "データ型はtrivially copyableである必要があります");
    const uint8_t * bytes = reinterpret_cast<const uint8_t *>(&data);
    buffer_.insert(buffer_.end(), bytes, bytes + sizeof(T));
  }

  template <typename T>
  T read()
  {
    static_assert(std::is_trivially_copyable_v<T>, "データ型はtrivially copyableである必要があります");
    if (position_ + sizeof(T) > buffer_.size()) {
      throw std::out_of_range("ストリームの末尾を超えて読み取りを試みました");
    }

    T result;
    std::memcpy(&result, buffer_.data() + position_, sizeof(T));
    position_ += sizeof(T);
    return result;
  }

  // バッファ全体を取得
  const std::vector<uint8_t> & getBuffer() const { return buffer_; }

private:
  std::vector<uint8_t> buffer_;
  size_t position_ = 0;
};
```

この実装のポイント：

1. **型安全性** - テンプレート関数と静的アサートにより、適切な型のみが使用されることを保証
2. **効率性** - バイナリレベルの処理により、シリアライズとデシリアライズを効率的に実行
3. **エラー処理** - 範囲外アクセスに対する例外を投げることで、安全性を確保

## 実装上の注意点

### 効率性の考慮

crane_basicsでは効率性を重視しており、以下の技術を使用しています：

1. **インライン関数** - 小さな関数はインライン化して関数呼び出しのオーバーヘッドを削減
2. **Move意味論** - 不必要なコピーを避けるためにC++のムーブセマンティクスを活用
3. **メモリアロケーション最適化** - 動的メモリ確保を最小限に抑える設計

### スレッドセーフティ

基本的に、crane_basicsのクラスと関数はスレッドセーフではありません。複数のスレッドからアクセスする場合は、適切な同期機構を実装する必要があります。例外として、以下のクラスは内部的な同期を行っています：

- `UDPSender` - Boost.Asioの内部同期メカニズムを利用
- `DiagnosedPublisher` - ROS 2のパブリッシャーが提供するスレッドセーフティに依存

### パフォーマンス最適化のヒント

crane_basicsパッケージを使用する際のパフォーマンス最適化のヒント：

1. **幾何学計算の順序** - 計算コストの低い操作（例：バウンディングボックスのチェック）を先に行い、詳細な交差計算などのコストの高い操作を後に行う
2. **PID制御器のチューニング** - 実際の使用環境でPIDパラメータを適切にチューニングすることで、制御性能を最大化
3. **メモリ管理** - `std::shared_ptr`の過剰な使用を避け、適切な場合は`std::unique_ptr`や直接オブジェクトを使用

### デバッグ手法

crane_basicsを使用したコードをデバッグする際の手法：

1. **可視化** - 幾何学的な計算結果の可視化により、問題の特定が容易に
2. **単体テスト** - 各コンポーネントに対する単体テストを作成して問題を早期に検出
3. **ロギング** - ROS 2のロギング機能を活用して、重要なイベントや値の変化をログに記録

## 将来の拡張計画

crane_basicsパッケージは今後以下の機能で拡張される予定です：

1. **より多様な幾何学モデル** - より複雑な形状（楕円、多角形など）のサポート
2. **3D幾何学演算** - 将来的に3D環境での操作をサポートするための拡張
3. **高度な制御アルゴリズム** - PID以外の制御アルゴリズム（MPC、LQRなど）の実装
4. **機械学習統合** - 機械学習モデルとの連携機能の追加

## まとめ

crane_basicsパッケージは、ロボットサッカーシステムに必要な基本機能を効率的かつ使いやすい形で提供しています。このドキュメントで説明した実装詳細を理解することで、パッケージの拡張や最適化が容易になり、より高度なロボット制御アルゴリズムの開発が可能になります。

コード貢献やバグ報告は、GitHubリポジトリを通じて歓迎されています。

コード貢献やバグ報告は、GitHubリポジトリを通じて歓迎されています。
