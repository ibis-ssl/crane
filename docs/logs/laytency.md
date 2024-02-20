# 遅延のボトルネック解析

<https://github.com/ibis-ssl/crane/issues/130>

## Timer/Callback駆動

まずは現状分析

```mermaid
graph TD
    LP["`Callback
    **Local Planner**
    Immediately`"]
    PS[Play Switcher]
    GA[Game Analyzer]

    subgraph interface
        VC["`100Hz
        **Vision Component**
        Immediately`"]
        GrC[GrSim Component]
        GCC[Game Controller Component]
        Receiver[Robot Receiver]
    end

    SC["`WorldModel Callback
    **Session Controller**
    Immediately`"]
    SS[Sim Sender]

    VT["`100Hz
        **Vision Tracker**
        Immediately`"]
    WP["`Callback
        **World Model Publisher**
        30Hz`"]

    subgraph software
        GrSim[GrSim]
        GC[Game Controller]
    end

    WP -- /world_model -->  SC
    WP -- /world_model -->  GA
    GA -- /game_analysis -->  SC


    GrC -. UDP .->  GrSim

    GrSim -. UDP .->  GrC
    GrC -- /geometry -->  WP

    GrSim -. UDP .->  VC


    VC -- /detection -->  VT
    VT -- /detection_tracked -->  WP

    GC -. UDP .->  GCC
    GCC -- /referee -->  PS
    PS -- /play_situation --> SC
    SC -- /control_targets --> LP
    LP -- /robot_commands -->  SS
    SS -- /commands -->  GrC

    Receiver -- /feedback -->  WP
```

- 下流はほとんどコールバック駆動なので良し
- Vision Component
  - 非同期で受け取ってUDPコールバックにしても良いかも
  - これは絶対に受け取った時刻を一緒に情報として持っておくべき
- Vision Tracker
  - フィルター処理はタイマー駆動必要かも
  - これ以降の処理はコールバックで一気通貫したい
    - 柔軟性的にはフィルター処理コールバック周期と出力周期は変えたいかも
    - いい感じ補間処理入れたら出力周期変えてもpublish時の最新の情報を出せる？
