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

使用するロボットコマンドの制御モードは複数あるため、ベースクラスのテンプレート引数で使用する制御モードのロボットコマンドを指定する必要がある。

```c++
class Kick : public SkillBase<RobotCommandWrapperPosition>
```

現状、以下のロボットコマンドが存在する。

- RobotCommandWrapperPosition
- RobotCommandWrapperSimpleVelocity

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

// テンプレートでロボットコマンドの型とステートの型を指定する
class TestSkill : public SkillBaseWithState<TestState, RobotCommandWrapperPosition>
{
public:
    TestSkill(RobotCommandWrapperBase::SharedPtr & base)
    // スキルの名前、初期ステートを指定する
    : SkillBaseWithState<TestState, RobotCommandWrapperPosition>("Test", base, TestState::STATE_1)
    {
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

### 複合スキルの実装

複数のスキルを組み合わせて、より複雑な動作を実現できる複合スキルも実装可能です。`Attacker`クラスは複合スキルの良い例で、内部に`Kick`、`GoalKick`、`Receive`などの基本スキルを保持し、状態に応じて適切なスキルに処理を委譲します。

```c++
// Attackerスキルの例
class Attacker : public SkillBaseWithState<AttackerState, RobotCommandWrapperPosition>
{
public:
  explicit Attacker(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBaseWithState<AttackerState, RobotCommandWrapperPosition>("Attacker", base, AttackerState::ENTRY_POINT),
    kick_target(getContextReference<Point>("kick_target")),
    forced_pass_receiver_id(getContextReference<int>("forced_pass_receiver_id")),
    kick_skill(base),
    goal_kick_skill(base),
    receive_skill(base)
  {
    // 状態ごとの処理を登録
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
};
```

## 可視化APIの使用方法

最新のスキル実装では、可視化APIが拡張され、メソッドチェーンによる直感的なインターフェースが利用できるようになりました。これにより、デバッグやスキルの状態表示がより簡単になりました。

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
  .from(robot()->pose.pos)
  .to(target_pos)
  .stroke("blue")
  .strokeWidth(2)
  .build();

// 矢印を描画
visualizer->arrow()
  .from(robot()->pose.pos)
  .to(target_pos)
  .stroke("green")
  .strokeWidth(2)
  .build();
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
{
  // パラメータの名前とデフォルト値を設定する
  // パラメータの名前は重複不可（重複した場合、上書きされる）
  // パラメータの型は自動で判別される
  setParameter("test_int_param", 0);
  // C++のバグで、stdL::stringの設定をする場合は明示的に型を指定する必要がある。
  // （指定しないとboolになってしまう）
  setParameter("test_string_param", std::string("test"));
  // 日本語の文字列も設定可能
  setParameter("テスト", std::string("テスト"));
}
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
class TestSkill{
private:
    // コンテキスト用のメンバ変数を宣言する
    // 参照型である必要がある
    int & context_int;
public:
    TestSkill(RobotCommandWrapperBase::SharedPtr & base)
    // コンテキスト用のメンバ変数を初期化する
    // SimpleAI上で表示する名前を指定する
    : context_int(getContextReference<int>("context_int"))
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

<https://github.com/ibis-ssl/crane/blob/develop/crane_simple_ai/src/crane_commander.cpp#L38>

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

### 最新の開発ガイドライン

スキル開発において最近採用されているベストプラクティスは以下の通りです：

1. **複合スキルの活用**: 基本スキルを組み合わせて複雑な動作を実現
2. **状態遷移の明確化**: 状態遷移図を設計してから実装に移る
3. **可視化APIの積極的活用**: デバッグと開発効率向上のため、ロボットの動きや状態を視覚的に表現
4. **コンテキストの活用**: 重要な内部変数をコンテキストとして登録し、SimpleAIでの監視を容易にする
5. **パラメータチューニング**: 実行時に調整可能なパラメータを設定し、実環境での最適化を容易にする

これらの手法を活用することで、より堅牢で柔軟なスキルを開発することが可能になります。
