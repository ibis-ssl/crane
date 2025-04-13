# Skill

Skillとは、単一のロボットの動作を表す単位である。

## スキルクラスの実装

スキルクラスは、`crane_robot_skills`パッケージに実装する。
`crane_robot_skills`パッケージは多くの被依存パッケージを持つため、ヘッダファイルを変更するとコンパイル時間が増大してしまう。
そのため、実装はなるべくcppファイルに書くようにすると良い。

### スキルの入出力

world_modelなどの情報を受けとり、ロボットコマンドを出力することで、ロボットの動きを制御する。
また、`crane_visualizer`向けに可視化情報を出力することもできる。

### ベースクラス

スキルクラスは、ベースクラスを継承して実装するが、用途に合わせてベースクラスは2種類存在する。

- crane::skills::SkillBase
- crane::skills::SkillBaseWithState

どちらのベースクラスも、`crane::skills::SkillInterface`クラスを継承している。

```c++
class Kick : public SkillBaseWithState<KickState>
{
public:
  template <typename... Args>
  explicit Kick(Args &&... args)
  : SkillBaseWithState<KickState>("Kick", std::forward<Args>(args)...),
    receive_skill(*this)
  {
    // パラメータ初期化
    initialize();
  }

  void initialize();  // 別ファイルで定義される初期化関数
};
```

#### crane::skills::SkillBase

標準のベースクラス。
最低限update関数の実装が必要である。  
オプションとしてprint関数を実装することもできる。

```c++
Status update() override
{
    // メンバーのcommandを使ってロボットを動かす
    Point pos{0,0};
    command.setTargetPosition(pos);
    // visualizerを使って可視化情報を出力する
    visualizer->circle()
      .position(pos)
      .radius(0.1)
      .fill("white")
      .build();
    // スキルの状態を返す（SUCCESS/FAILUREになると終了）
    return Status::RUNNING;
}
```

#### crane::skills::SkillBaseWithState

状態遷移を行う動きを作りたい場合に使える、ステートマシンを組み込んだベースクラス。  
update関数の実装の代わりに、コンストラクタでステートごとのupdate関数と状態遷移条件の設定を行う。

```c++
// enum classでステートを定義する
enum class TestState
{
    STATE_1,
    STATE_2,
};

// テンプレートでステートの型を指定する
class TestSkill : public SkillBaseWithState<TestState>
{
public:
    template <typename... Args>
    explicit TestSkill(Args &&... args)
    // スキルの名前とパラメータを転送
    : SkillBaseWithState<TestState>("Test", std::forward<Args>(args)...)
    {
        // initialize()関数を実装して呼び出すパターンも一般的
        // 単純な場合はコンストラクタ内に直接実装してもOK

        // ステートごとのupdate関数を登録する
        addStateFunction(TestState::STATE_1,
            [this]() -> Status {
              // STATE_1の処理
              return Status::RUNNING;
        });

        addStateFunction(TestState::STATE_2,
            [this]() -> Status {
              // STATE_2の処理
              return Status::RUNNING;
        });

        // ステート遷移条件を登録する
        // 遷移条件は毎フレーム評価され、条件がTRUEになると遷移する。
        // 登録順に評価されるので、最初にTRUEになった条件で遷移することに注意。
        // （優先度が高い遷移条件から順番に設定すると良い）
        addTransition(TestState::STATE_1, TestState::STATE_2, [this]() -> bool {
            // STATE_1からSTATE_2への遷移条件
            return true;  // この場合、必ず遷移する
        });
    }
};
```

### スキルコマンドとWorldModelの利用

スキルの中では、`command`および`world_model()`メソッドを使ってロボットコマンドの生成とワールドモデルへのアクセスができます。

```c++
// ロボットの位置を設定
command.setTargetPosition(target_pos);

// ドリブルパワーを設定
command.getEditableMsg().dribble_power = 0.8;

// キックパワーを設定
command.getEditableMsg().kick_power = 0.5;

// チップキックを有効にする
command.getEditableMsg().chip_enable = true;

// 自分のロボット情報にアクセス
auto robot_pos = robot()->pose.pos;

// ボール情報にアクセス
auto ball_pos = world_model()->ball.pos;

// 敵ゴールの位置を取得
auto their_goal = world_model()->getTheirGoalCenter();
```

### 複合スキルの実装

複数のスキルを組み合わせて、より複雑な動作を実現できる複合スキルも実装可能です。`Attacker`クラスは複合スキルの良い例で、内部に`Kick`、`GoalKick`、`Receive`などの基本スキルを保持し、状態に応じて適切なスキルに処理を委譲します。

```c++
// Attackerスキルの例
class Attacker : public SkillBaseWithState<AttackerState>
{
public:
  template <typename... Args>
  explicit Attacker(Args &&... args)
  : SkillBaseWithState<AttackerState>("Attacker", std::forward<Args>(args)...),
    kick_target(getContextReference<Point>("kick_target")),
    forced_pass_receiver_id(getContextReference<int>("forced_pass_receiver")),
    kick_skill(*this),  // 自分自身を内部スキルに渡す
    goal_kick_skill(*this),
    receive_skill(*this)
  {
    // コンストラクタ内での実装は最小限に抑え、initialize()でまとめて初期化する
    initialize();
  }

  void initialize() {
    // パラメータの初期化
    setParameter("moving_ball_velocity", 1.0);

    // 内部スキルの設定
    receive_skill.setParameter("policy", std::string("closest"));

    // 状態関数とトランジションの登録
    addStateFunction(AttackerState::ENTRY_POINT, [this]() -> Status {
      // エントリーポイントの処理...
      return Status::RUNNING;
    });

    addStateFunction(AttackerState::KICK, [this]() -> Status {
      // キックスキルに処理を委譲
      return kick_skill.run();
    });

    // 状態遷移条件を設定
    addTransition(AttackerState::ENTRY_POINT, AttackerState::KICK, [this]() -> bool {
      // ENTRY_POINTからKICKへの遷移条件
      return /* 条件 */;
    });
  }

  // 内部スキル
  Kick kick_skill;
  GoalKick goal_kick_skill;
  Receive receive_skill;

  // コンテキスト変数
  Point & kick_target;
  int & forced_pass_receiver_id;

  // その他のメンバ変数
  std::optional<uint8_t> pass_receiver_id = std::nullopt;
};
```

## 可視化APIの使用方法

スキル実装では、可視化APIがメソッドチェーンによる直感的なインターフェースを提供しています。これにより、デバッグやスキルの状態表示がより簡単になりました。

```c++
// 円を描画
visualizer->circle()
  .position(pos)
  .radius(0.1)
  .fill("white")
  .build();

// テキストを表示
visualizer->text()
  .position(robot()->pose.pos)
  .text("状態: " + state_string)
  .fontSize(50)
  .fill("white")
  .build();

// 線を描画
visualizer->line()
  .start(robot()->pose.pos)
  .end(target_pos)
  .stroke("blue")
  .strokeWidth(2)
  .build();

// 矢印を描画
visualizer->arrow()
  .start(robot()->pose.pos)
  .end(target_pos)
  .stroke("green")
  .strokeWidth(2)
  .build();

// 折れ線を描画
auto polyline_builder = visualizer->polyline();
for (auto point : points) {
  polyline_builder.addPoint(point);
}
polyline_builder.stroke("orange", 0.3).strokeWidth(2).build();
```

## スキルのパラメータ

SimpleAIやスキルを使用するクラスなどで、スキルのパラメータを自由に設定できる。
SimpleAIの画面で自由に設定できるので、デバッグや調整が容易になる。

### パラメータの型

現在、以下の型が使用可能である。

- int
- double
- std::string
- bool
- Point

### パラメータの宣言とデフォルト値設定

スキルクラスのコンストラクタで、パラメータを受け取る変数を宣言し、デフォルト値を設定する。

```c++
// パラメータの名前とデフォルト値を設定する
// パラメータの名前は重複不可（重複した場合、上書きされる）
// パラメータの型は自動で判別される
setParameter("test_int_param", 0);
// 注意：std::stringの設定をする場合は明示的に型を指定する必要がある。
// （指定しないとboolになってしまう場合がある）
setParameter("test_string_param", std::string("test"));
// 日本語の文字列も設定可能
setParameter("テスト", std::string("テスト"));
```

### パラメータの上書き

setParameter関数を使って、パラメータを上書きできる。
宣言したときの型と異なる型で上書きすると、エラーが発生する（はず）。

```c++
setParameter("test_int_param", 1);
```

### パラメータの取得

getParameter関数を使って、パラメータを取得することができる。

```c++
// テンプレートで型を指定する
// パラメータが存在しなかったり、型が異なる場合は、例外が発生する
int test_int_param = getParameter<int>("test_int_param");
```

## スキルのコンテキスト

スキルの内部変数をコンテキストとして登録することで、SimpleAIで内部変数の値を表示できるようになるので、
デバッグが容易になる。

SimpleAI上では、コンテキストの値を表示できるが、コンテキストの値を変更することはできない。

### コンテキストの型

現在、以下の型が使用可能である。

- int
- double
- std::string
- bool
- Point
- std::optional<Point>

### コンテキストの設定

```c++
class TestSkill : public SkillBase {
private:
    // コンテキスト用のメンバ変数を宣言する
    // 参照型である必要がある
    int & context_int;
public:
    template <typename... Args>
    explicit TestSkill(Args &&... args)
    // スキルの名前を指定し、引数を転送
    : SkillBase("Test", std::forward<Args>(args)...),
      context_int(getContextReference<int>("context_int"))
    {}

    Status update() override
    {
        // コンテキストの値を変更する（普通の変数同様に読み書きしてOK）
        context_int = 1;
        return Status::RUNNING;
    }
};
```

## スキルをSimpleAIで使えるようにする

実装したスキルをSimpleAIで使えるようにするには、いくつかの手順が必要である。

### スキルのヘッダファイルを追加

SimpleAIでスキルを一括インクルードするためのヘッダファイルに作成したスキルのヘッダファイルを追加する。
<https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/skills.hpp>

### スキルを登録する

<https://github.com/ibis-ssl/crane/blob/develop/crane_simple_ai/src/crane_commander.cpp>

setUpSkillDictionary関数でスキルを登録する。

```c++
setUpSkillDictionary<skills::TestSkill>();
```

作ったスキルすべてを登録すると、SimpleAIのスキル選択プルダウンの表示が大変なことになるので、あまり使わないスキルはコメントアウトしてある。

## スキルをセッションに組み込む

### 対応するPlannerを作る

スキルは直接セッションで動かせないため、スキル用のPlannerを作る必要がある。
Skill単体のPlannerはskill_planner.hppに実装することが多い。

<https://github.com/ibis-ssl/crane/blob/develop/session/crane_planner_plugins/include/crane_planner_plugins/skill_planner.hpp>

`DEFINE_SKILL_PLANNER`マクロを使用すると、簡単にスキルのプランナーを定義できます。

```c++
DEFINE_SKILL_PLANNER(TestSkill)  // TestSkillPlannerというクラスが定義される
```

より複雑なプランナーが必要な場合は、個別にクラス定義ファイルを作ります（例：AttackerSkillPlanner）。

### Plannerの登録

crane_planner_plugins/planners.hppに文字列とPlannerのペアを登録する

<https://github.com/ibis-ssl/crane/blob/develop/session/crane_planner_plugins/include/crane_planner_plugins/planners.hpp>

```c++
  } else if (planner_name == "test_skill") {
    return std::make_shared<TestSkillPlanner>(ts...);
  }
```

### セッションでの呼び出し

セッションファイルで登録した文字列を使ってPlannerを呼び出す。

```yaml
name: test
description: TestSkillのためのセッション
sessions:
  - name: test_skill
    capacity: 1
  - name: waiter
    capacity: 20
```

セッションファイルは以下のディレクトリにある
<https://github.com/ibis-ssl/crane/tree/develop/session/crane_session_controller/config/play_situation>

## 実践的なスキル開発事例

### Attackerスキルの状態遷移

Attackerスキルは複合スキルの好例で、内部で状態遷移を行いながら複数のサブスキルを組み合わせます。最新のAttackerスキルでは以下の状態が定義されています：

```c++
enum class AttackerState {
  ENTRY_POINT,  // 初期状態
  FORCED_PASS,  // 強制パス
  RECEIVE,      // ボールを受け取る
  KICK,         // キックする
  FINAL_GUARD,  // 最終防御
};
```

これらの状態間の遷移は条件によって定義され、各状態では適切なサブスキル（Kick、GoalKick、Receiveなど）が実行されます。たとえば、ボールを受け取るべき状況では`RECEIVE`状態に遷移し、内部の`receive_skill`が実行されます。状況に応じて最適なキック方法（ゴールキック、標準パスなど）が選択されます。

### スキル実装の詳細例

#### 実際のコード例：ボールの受け取り条件判定

```c++
addTransition(AttackerState::ENTRY_POINT, AttackerState::RECEIVE, [this]() -> bool {
  // ボールが遠くにいる/動いている/自分に向かってきている
  if (
    robot()->getDistance(world_model()->ball.pos) > 1.0 &&
    world_model()->ball.isMoving(getParameter<double>("moving_ball_velocity")) &&
    world_model()->ball.isMovingTowards(robot()->pose.pos)) {
    return true;
  } else {
    return false;
  }
});
```

#### 実際のコード例：パス受け手の選択

```c++
std::shared_ptr<RobotInfo> Attacker::selectPassReceiver()
{
  auto our_robots = world_model()->ours.getAvailableRobots(robot()->id, true);
  const auto enemy_robots = world_model()->theirs.getAvailableRobots();
  double best_score = 0.0;
  std::shared_ptr<RobotInfo> best_bot = nullptr;
  for (auto & our_robot : our_robots) {
    Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
    auto target = our_robot->pose.pos;
    double score = 1.0;

    // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
    auto [best_angle, goal_angle_width] = world_model()->getLargestGoalAngleRangeFromPoint(target);
    score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);

    // 敵ゴールに近いときはスコアを上げる
    double normed_distance_to_their_goal = ((target - world_model()->getTheirGoalCenter()).norm() -
                                          (world_model()->field_size.x() * 0.5)) /
                                         (world_model()->field_size.x() * 0.5);
    // マイナスのときはゴールに近い
    score *= (1.0 - normed_distance_to_their_goal);

    // パスラインの障害物チェック
    if (auto nearest_enemy =
          world_model()->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
        nearest_enemy) {
      // ボールから遠い敵がパスコースを塞いでいる場合は諦める
      if (
        nearest_enemy->robot->getDistance(world_model()->ball.pos) > 1.0 &&
        nearest_enemy->distance < 0.4) {
        score = 0.0;
      }
      // パスラインに敵がいるときはスコアを下げる
      score *= 1.0 / (1.0 + nearest_enemy->distance);
    }

    if (score > best_score) {
      best_score = score;
      best_bot = our_robot;
    }
  }

  return best_bot;
}
```

### 最新の開発ガイドライン

スキル開発において最近採用されているベストプラクティスは以下の通りです：

1. **複合スキルの活用**: 基本スキルを組み合わせて複雑な動作を実現
2. **状態遷移の明確化**: 状態遷移図を設計してから実装に移る
3. **可視化APIの積極的活用**: デバッグと開発効率向上のため、ロボットの動きや状態を視覚的に表現
4. **コンテキストの活用**: 重要な内部変数をコンテキストとして登録し、SimpleAIでの監視を容易にする
5. **パラメータチューニング**: 実行時に調整可能なパラメータを設定し、実環境での最適化を容易にする

これらの手法を活用することで、より堅牢で柔軟なスキルを開発することが可能になります。

## フレームワークの最新機能

### 状態遷移の改善

現在のフレームワークでは、ENTRY_POINTという特別な状態を使用して状態初期化や遷移判定を行います。注意点として、ENTRY_POINTのstate functionは実行されず、遷移判定のみ行われます。これにより、状態初期化と実行を明確に分離できます。

```c++
// "ENTRY_POINT"のstate functionは実行されない（skill_base.hppのStateMachine::update参照）
// ので自分への遷移関数で初期化処理を実装
addTransition(AttackerState::ENTRY_POINT, AttackerState::ENTRY_POINT, [this]() -> bool {
  pass_receiver_id = std::nullopt;
  return false;
});
```

### ロボット選択ロジック

よりインテリジェントなロボット選択ロジックが実装されています。たとえば、AttackerSkillPlannerでは、「フロントライン上のロボット」やスコア関数による選択が行われます。

```c++
auto getSelectedRobots(...) -> std::vector<uint8_t> override
{
  if (auto our_frontier = world_model->getOurFrontier();
      our_frontier && ranges::contains(selectable_robots, our_frontier->robot->id)) {
    // フロントライン上のロボットを選択
    skill = std::make_shared<skills::Attacker>("attacker", our_frontier->robot->id, world_model);
    return {our_frontier->robot->id};
  } else {
    // ボールに一番近いロボットを選択（スコア関数を使用）
    auto selected_robots = this->getSelectedRobotsByScore(
      1, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
      },
      prev_roles, context);

    if (!selected_robots.empty()) {
      // 選択したロボットIDでスキルを初期化
      skill = std::make_shared<skills::Attacker>(selected_robots.front(), world_model);
      return selected_robots;
    }
    return {};
  }
}
```

### セッション設定

セッションファイル（YAML）でプランナーの組み合わせを定義できます。例えば、INPLAYセッションは複数のプランナーを組み合わせています：

```yaml
name: INPLAY
description: INPLAY
sessions:
  - name: emplace_robot
    capacity: 1
  - name: total_defense
    capacity: 2
  - name: attacker_skill
    capacity: 1
  - name: pass_receive
    capacity: 1
  - name: sub_attacker_skill
    capacity: 1
  - name: marker
    capacity: 2
  - name: simple_placer
    capacity: 20
```

各プランナーには最大何台のロボットを割り当てるかを示す`capacity`パラメータがあります。これにより、チーム全体の戦略を柔軟に設定できます。
