# ノードダイアグラム

## シミュレータ環境

```mermaid
graph TD
    LP[Local Planner]
    PS[Play Switcher]
    GA[Game Analyzer]

    subgraph interface
        VC[Vision Component]
        GrC[GrSim Component]
        GCC[Game Controller Component]
        Receiver[Robot Receiver]
    end

    SC[Session Controller]
    SS[Sim Sender]

    VT[Vision Tracker]
    WP[World Model Publisher]

    subgraph software
        GrSim[GrSim]
        GC[Game Controller]
    end

    WP -- /world_model -->  SC
    WP -- /world_model -->  GA
    GA -- /game_analysis -->  SC


    GrC -. UDP .->  GrSim

    GrSim -. UDP .->  VC
    VC -- /geometry -->  WP

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

## 実機環境

```mermaid
graph TD

    subgraph robot
        Robots[Robots]
    end

    subgraph AI Computer
        LP[Local Planner]
        PS[Play Switcher]

        VC[Vision Component]
        GCC[Game Controller Component]
        Receiver[Robot Receiver]

        SC[Session Controller]
        SS[Real Sender]

        VT[Vision Tracker]
        WP[World Model Publisher]
    end

    subgraph software
        Vision[SSL Vision]
        GC[Game Controller]
    end

    WP -- /world_model -->  SC
    WP -- /world_model -->  PS
    WP -- /world_model -->  LP



    Robots -. UDP .-> Receiver
    VC -- /geometry -->  WP

    Vision -. UDP .->  VC


    VC -- /detection -->  VT
    VT -- /detection_tracked -->  WP

    GC -. UDP .->  GCC
    GCC -- /referee -->  PS
    PS -- /play_situation --> SC
    PS -- /play_situation --> WP
    SC -- /control_targets --> LP
    LP -- /robot_commands -->  SS
    SS -. UDP .->  Robots

    Receiver -- /feedback -->  WP
```

## テスト用のノードダイアグラム

```mermaid
graph TD
    subgraph interface
        VisionNode[Vision Component]
        Sender[Real Sender]
        Receiver[Robot Receiver]
    end

    VT[Vision Tracker]
    WP[World Model Publisher]
    Main[Simple AI]
    LP[Local Planner]

    subgraph RealWorld
        Robot[Actual Robot CM4]
        SSLVision[SSL Vision]
    end

    SSLVision -. UDP .->  VisionNode
    VisionNode -- /detection -->  VT
    VT -- /detection_tracked -->  WP
    VisionNode -- /geometry -->  WP

    WP -- /world_model -->  Main
    Main -- /control_targets --> LP
    LP -- /robot_commands -->  Sender

    Sender -. UDP .->  Robot

    Robot -. UDP .->  Receiver
    Receiver -- /feedback -->  WP
```
