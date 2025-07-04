# crane_basics パッケージ 高度な使用法

このドキュメントでは、crane_basicsパッケージの高度な使用法と詳細な機能について説明します。

## 幾何学モデルと演算

### カスタム幾何学モデル

crane_basicsは、Boost.Geometryと互換性のあるカスタム幾何学モデルを提供します。

#### Circle（円）モデル

```cpp
namespace crane::geometry::model
{
template <typename Point>
struct Circle
{
  Point center;
  double radius;
};
}
```

このモデルはBoost.Geometryの標準モデルとして扱われるよう、以下の特殊化も提供されています：

```cpp
namespace boost::geometry::traits
{
// Circleの特性を定義
template <typename Point>
struct tag<crane::geometry::model::Circle<Point>>
{
  using type = void;  // circleタグ
};

// 距離計算アルゴリズムなどの特殊化
// ...
}
```

#### Capsule（カプセル）モデル

カプセルは、円柱の両端に半球がついた形状です：

```cpp
namespace crane::geometry::model
{
template <typename PointType>
struct Capsule
{
  // PointType first; // 旧定義
  // PointType second; // 旧定義
  boost::geometry::model::segment<PointType> segment; // 線分 (両端点を含む)
  double radius; // 半径

  // 線分の端点には segment.first, segment.second でアクセス
};
}
```

### 複雑な交差判定

`getIntersections`関数群は様々な幾何学形状間の交点を計算します：

```cpp
// 線分と線分の交点
auto intersections = getIntersections(segment1, segment2);

// 円と線分の交点
auto intersections = getIntersections(circle, segment);

// その他の幾何形状の交点（テンプレート関数）
auto intersections = getIntersections(geometry1, geometry2);
```

特に、円と線分の交点計算は以下のアルゴリズムを使用します：

1. 円と線分の距離を計算
2. 距離が円の半径より大きい場合、交点なし
3. 交点の座標を解析的に計算
4. 交点が線分上にあるかを確認

```cpp
inline auto getIntersections(const Circle & circle, const Segment & segment) -> std::vector<Point>
{
  std::vector<Point> intersections;
  double distance = bg::distance(circle, segment);
  if (distance > circle.radius) {
    // 交差しない
    return intersections;
  } else {
    // 交差する - 交点計算のアルゴリズム
    // ...
  }
}
```

## ヒステリシス機能

`Hysteresis`クラスは状態変化に履歴依存性を持たせるために使用されます。これは、センサ値が小さな変動で状態が頻繁に切り替わることを防ぐのに役立ちます。

```cpp
struct Hysteresis
{
  Hysteresis(double lower, double upper) : lower_threshold(lower), upper_threshold(upper) {}

  double lower_threshold;
  double upper_threshold;
  bool is_high = false;

  std::function<void(void)> upper_callback = []() {};
  std::function<void(void)> lower_callback = []() {};

  void update(double value);
};
```

使用方法：

```cpp
// ボールの速度に基づくヒステリシス（0.1m/s以下で「停止」、0.6m/s以上で「移動中」と判定）
Hysteresis ball_speed_hysteresis(0.1, 0.6);

// コールバックを設定
ball_speed_hysteresis.upper_callback = []() {
  std::cout << "ボールが移動開始しました" << std::endl;
};
ball_speed_hysteresis.lower_callback = []() {
  std::cout << "ボールが停止しました" << std::endl;
};

// 値を更新（このとき上記コールバックが条件に応じて呼ばれる）
ball_speed_hysteresis.update(ball_velocity.norm());
```

## ノードハンドル高度な使用法

`NodeHandle`クラスはROS 2ノードの各種インターフェースへのアクセスを簡略化します。

### 特定インターフェースの取得

```cpp
// 必要なインターフェースを指定してNodeHandleを作成
using MyNodeHandle = crane::NodeHandle<
  rclcpp::node_interfaces::NodeBaseInterface,
  rclcpp::node_interfaces::NodeTopicsInterface,
  rclcpp::node_interfaces::NodeTimersInterface
>;

// 既存のノードからハンドルを作成
auto node = std::make_shared<rclcpp::Node>("my_node");
MyNodeHandle handle(*node);

// 特定のインターフェースを取得して使用
auto topics_interface = handle.get_interface<rclcpp::node_interfaces::NodeTopicsInterface>();
auto pub = topics_interface->create_publisher<std_msgs::msg::String>("topic", 10);
```

### 複合コンポーネントでの使用例

```cpp
class MyComponent : public rclcpp::Node
{
public:
  MyComponent(const rclcpp::NodeOptions & options)
  : Node("my_component", options),
    node_handle_(*this)
  {
    // ノードハンドルを使用してパブリッシャーを作成
    auto topics = node_handle_.get_interface<rclcpp::node_interfaces::NodeTopicsInterface>();
    pub_ = topics->create_publisher<std_msgs::msg::String>("topic", 10);

    // タイマーを作成
    auto timers = node_handle_.get_interface<rclcpp::node_interfaces::NodeTimersInterface>();
    timer_ = timers->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { timer_callback(); });
  }

private:
  using NodeHandleT = crane::NodeHandle<
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeTimersInterface
  >;

  NodeHandleT node_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello";
    pub_->publish(std::move(msg));
  }
};
```

## ボール物理モデル

ボールの物理モデルは、実際のボールの動きをシミュレートし、将来位置を予測するのに役立ちます。

### 減速を考慮したボール軌道予測

`getFutureBallPosition`関数は、摩擦による減速を考慮してボールの将来位置を計算します：

```cpp
inline Point getFutureBallPosition(
  Point ball_pos, Point ball_vel, double t, double deceleration = 0.5)
{
  // 指定時間までに停止する場合
  if (ball_vel.norm() - deceleration * t < 0.) {
    double stop_time = ball_vel.norm() / deceleration;
    return ball_pos + ball_vel * stop_time -
           0.5 * stop_time * stop_time * deceleration * ball_vel.normalized();
  } else {
    return ball_pos + ball_vel * t - 0.5 * t * t * deceleration * ball_vel.normalized();
  }
}
```

この関数は以下の物理モデルに基づいています：

- ボールの摩擦による減速は一定（デフォルトでは0.5 m/s²）
- 減速方向はボールの速度ベクトルと逆向き
- 運動方程式: x = x₀ + v₀t - 0.5at²

### ボール位置シーケンスの生成

将来のボール位置を時間間隔ごとに取得するには、`getBallSequence`関数を使用します：

```cpp
// ボールの現在位置と速度
Point ball_pos(1.0, 0.0);
Point ball_vel(2.0, 1.0);

// 3秒間、0.1秒間隔でボール位置を予測
auto sequence = getBallSequence(3.0, 0.1, ball_pos, ball_vel);

// 結果を使用
for (const auto & [pos, time] : sequence) {
  std::cout << "Time: " << time << "s, Position: ("
            << pos.x() << ", " << pos.y() << ")" << std::endl;
}
```

## ROS 2パブリッシャー診断機能

`DiagnosedPublisher`クラスは、ROS 2パブリッシャーに診断機能を追加します。これにより、パブリッシャーのパフォーマンス（発行頻度、遅延など）を監視できます。

```cpp
template <typename MessageT>
class DiagnosedPublisher
{
public:
  // コンストラクタのシグネチャが変更されています
  // template <typename NodePointerT>
  // DiagnosedPublisher(
  //   const NodePointerT node, const std::string & topic_name, const size_t qos_history_depth,
  //   double min_update_frequency, double max_update_frequency,
  //   const diagnostic_updater::TimeStampStatusParam & stamp =
  //     diagnostic_updater::TimeStampStatusParam())
  // 実際のコンストラクタに合わせて修正（詳細は `diagnosed_publisher.hpp` を参照）
  DiagnosedPublisher(
    /* 引数は実際のコードに合わせて調整 */
    // rclcpp::Publisher<MessageT>::SharedPtr publisher, // 旧シグネチャ例
    // std::shared_ptr<diagnostic_updater::Updater> updater, // 旧シグネチャ例
    // const std::string & name // 旧シグネチャ例
    // ... node, topic_name, qos_history_depth, etc.
  )
    // : publisher_(publisher), updater_(updater) // 旧初期化子リスト
  {
    // 診断タスクのセットアップ
    // ...
  }

  void publish(std::unique_ptr<MessageT> message)
  {
    // 発行時間を記録
    last_publish_time_ = publisher_->get_clock()->now();

    // 実際に発行
    publisher_->publish(std::move(message));

    // 診断情報を更新
    publish_count_++;
    // ...
  }

private:
  rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::Time last_publish_time_;
  uint64_t publish_count_ = 0;
  // ...
};
```

使用例：

```cpp
// 診断アップデーターとパブリッシャーの作成
// auto updater = std::make_shared<diagnostic_updater::Updater>(node); // これはDiagnosedPublisher内部で作成される場合がある
auto diagnosed_pub = std::make_shared<crane::DiagnosedPublisher<std_msgs::msg::String>>(
  node, // Nodeのポインタまたは参照
  "topic_name", // トピック名
  10, // QoS履歴深度
  0.1, // 最小更新頻度
  10.0 // 最大更新頻度
  // TimeStampStatusParam はオプション
);

// メッセージを発行
auto msg = std::make_unique<std_msgs::msg::String>();
msg->data = "Hello, World!";
diagnosed_pub->publish(std::move(msg));
```

## 詳細なトラベルタイム計算

`getTravelTimeTrapezoidal`関数は、台形速度プロファイルを考慮したロボットの移動時間を計算します。これは、加速区間、定速区間、減速区間を考慮した現実的な時間計算です。

```cpp
inline double getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration = 2.,
  const double max_velocity = 4.)
{
  double distance = (target - robot->pose.pos).norm();
  double initial_vel = robot->vel.linear.norm();

  // 加速・減速にかかる時間
  double accel_time = (max_velocity - initial_vel) / max_acceleration;
  double decel_time = max_velocity / max_acceleration;

  // 加速・減速にかかる距離
  double accel_distance = (initial_vel + max_velocity) * accel_time / 2;
  double decel_distance = max_velocity * decel_time / 2;

  if (accel_distance + decel_distance >= distance) {
    // 加速距離と減速距離の合計が移動距離を超える場合、定速区間はない
    // （数学的導出は省略）
    return (-initial_vel +
            2 * sqrt(0.5 * initial_vel * initial_vel + max_acceleration * distance)) /
           max_acceleration;
  } else {
    // 定速区間が存在する場合
    double remaining_distance = distance - (accel_distance + decel_distance);
    double cruise_time = remaining_distance / max_velocity;
    return accel_time + cruise_time + decel_time;
  }
}
```

このアルゴリズムの特徴：

1. ロボットの現在速度を考慮
2. 加速・減速の物理的制約を考慮
3. 短距離と長距離の両方に対応（短距離では定速区間なし）

実際の使用例：

```cpp
// ロボット情報
auto robot = std::make_shared<crane::RobotInfo>();
robot->pose.pos = crane::Point(0, 0);
robot->vel.linear = crane::Point(1.0, 0);  // 初速1.0m/s

// 目標位置
crane::Point target(5.0, 3.0);

// 移動時間計算
double travel_time = crane::getTravelTimeTrapezoidal(
  robot, target, 2.0, 3.0);  // 最大加速度2.0m/s², 最大速度3.0m/s

std::cout << "移動予測時間: " << travel_time << "秒" << std::endl;
```

## 詳細なボール接触検出 (修正)

**注意:** 以前このドキュメントで解説されていた状態（`HAVE`, `JUST_LOST`, `NOT_HAVE`）を持つ`BallContact`クラスは、現在の`utility/crane_basics/include/crane_basics/ball_contact.hpp`の実装とは異なります。
現在の`ball_contact.hpp`に定義されている`struct BallContact`は、よりシンプルな構造で、主に接触のタイムスタンプ（`last_contact_start_time`, `last_contact_end_time`）を管理します。
`RobotInfo`構造体内の`ball_contact`メンバーはこのシンプルなバージョンを使用します。

以下は、現在のコードベースに存在する`BallContact`の概念的な使用法です（実際のAPIは`ball_contact.hpp`を参照してください）：

```cpp
// utility/crane_basics/include/crane_basics/ball_contact.hpp の BallContact 構造体
struct BallContact
{
  std::chrono::system_clock::time_point last_contact_end_time;
  std::chrono::system_clock::time_point last_contact_start_time;
  // ... 他のメンバーやメソッド

  // 例: 接触があったかどうかを更新するメソッド
  void update(bool is_currently_in_contact);

  // 例: 接触時間や最終接触時刻を取得するメソッド
  std::chrono::duration<double> getContactDuration() const;
  bool findPastContact(double duration_sec) const;
};

// RobotInfo内での使用 (概念)
struct RobotInfo {
  // ...
  crane::BallContact ball_contact; // シンプルなタイムスタンプベースの BallContact
  // ...

  void some_update_method(bool is_ball_close_enough) {
    ball_contact.update(is_ball_close_enough); // is_ball_close_enough は接触判定の結果
  }
};
```

このため、以前記載されていた状態ベースの接触判定ロジック（カウンターや`hasBall()` / `justLost()`メソッドなど）は、`crane_basics`パッケージの現在の`BallContact`には直接的には含まれていません。そのようなロジックが必要な場合は、この基本的な`BallContact`の情報を用いて上位のモジュールで実装する必要があります。

## まとめ

crane_basicsパッケージは、RoboCup SSLプロジェクトにおける基本的なユーティリティを提供します。これらのユーティリティは、ロボットサッカーに必要な幾何学計算、物理モデル、ROS 2との連携などの機能を包括的に提供しています。

このパッケージを適切に使用することで、複雑なロボット制御アルゴリズムを効率的に実装することができます。特に、ボール予測、ロボット移動時間計算、幾何学演算などの機能は、高度な戦略アルゴリズム実装の基盤となります。
