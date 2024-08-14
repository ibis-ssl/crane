# Skill

Skillとは、単一のロボットの動作を表す単位である。

## スキルクラスの実装

スキルクラスは、`crane_robot_skills`パッケージに実装する。
`crane_robot_skills`パッケージは多くの被依存パッケージを持つため、ヘッダファイルを変更するとコンパイル時間が増大してしまう。
そのため、実装はなるべくcppファイルに書くようにすると良い。

### スキルの入出力

world_modelなどの情報を受けとり、ロボットコマンドを出力することで、ロボットの動きを制御する。
また、`consai_visualizer`向けに可視化情報を出力することもできる。

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
Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
{
    // メンバーのcommandを使ってロボットを動かす
    Point pos{0,0};
    command.setTargetPosition(pos);
    // visualizerを使って可視化情報を出力する
    visualizer->addCircle(pos, 0.1, 1, "white", "");
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
            [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
              // STATE_1の処理
              return Status::RUNNING;
        });
        
        addStateFunction(TestState::STATE_2,
            [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
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

## スキルのパラメータ

SimpleAIやスキルを使用するクラスなどで、スキルのパラメータを自由に設定することができる。
SimpleAIの画面で自由に設定することができるので、デバッグや調整が容易になる。

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

setParamter関数を使って、パラメータを上書きすることができる。
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

SimpleAI上では、コンテキストの値を表示することができるが、コンテキストの値を変更することはできない。

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
    : context_int(getContextReference<Point>("context_int"))
    {}
    
    Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
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
https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/skills.hpp


### スキルを登録する

https://github.com/ibis-ssl/crane/blob/develop/crane_simple_ai/src/crane_commander.cpp#L38

setUpSkillDictionary関数でスキルを登録する。

```c++
setUpSkillDictionary<skills::TestSkill>();
```


作ったスキルすべてを登録すると、SimpleAIのスキル選択プルダウンの表示が大変なことになるので、あまり使わないスキルはコメントアウトしてある。



## スキルをセッションに組み込む

### 対応するPlannerを作る

スキルは直接セッションで動かせないため、スキル用のPlannerを作る必要がある。
Skill単体のPlannerはskill_planner.hppに実装することが多い。

https://github.com/ibis-ssl/crane/blob/develop/session/crane_planner_plugins/include/crane_planner_plugins/skill_planner.hpp

### Plannerの登録

crane_planner_plugins/planners.hppに文字列とPlannerのペアを登録する

https://github.com/ibis-ssl/crane/blob/develop/session/crane_planner_plugins/include/crane_planner_plugins/planners.hpp

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
https://github.com/ibis-ssl/crane/tree/develop/session/crane_session_controller/config/play_situation
