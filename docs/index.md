---
title: ホーム
hide:
  - navigation
  - toc
---

<div class="ibis-hero" markdown>

<span class="ibis-hero__eyebrow">RoboCup Soccer Small Size League</span>

# ibis 公開ドキュメント

RoboCup SSL に出場するチーム ibis が、Orion ロボットの仕様と AI「Crane」などの開発で得た知見を公開しています。

[仕様を読む :lucide-arrow-right:](specs/README.md){ .md-button .md-button--primary }
[チームHP](https://www.ibis-ssl.tokyo/){ .md-button }

</div>

<div class="ibis-lead" markdown>

個人情報以外の情報は基本的にこのサイトに蓄積しています。外部の方だけでなく自分たちのメモも兼ねているため、文脈を理解していないと分からない内容があるかもしれません。
その代わり情報量はかなりありますので、既に SSL に出場している方からこれから出場しようと思っている方まで、多くの方の助けになると思います。

</div>

## 仕様

Orion ロボットの仕様の正本です。[ibis-ssl/specs](https://github.com/ibis-ssl/specs) から生成しています。

<div class="grid cards" markdown>

-   :lucide-network:{ .lg .middle .ibis-icon } __共通仕様__

    ---

    システム構成、制御・設定・フィードバックパケット、停止判定、ファームウェア更新。

    [:lucide-arrow-right: システム構成へ](specs/common/system_overview.md)

-   :lucide-cpu:{ .lg .middle .ibis-icon } __CM4__

    ---

    CM4 上で動くブリッジ、位置制御、カメラ。

    [:lucide-arrow-right: CM4 の概要へ](specs/cm4/overview.md)

-   :lucide-circuit-board:{ .lg .middle .ibis-icon } __Main 基板（STM32G474）__

    ---

    ハードウェア仕様、速度制御、ブートローダー、CM4 との UART、診断機能。

    [:lucide-arrow-right: Main 基板の概要へ](specs/stm/overview.md)

-   :lucide-monitor:{ .lg .middle .ibis-icon } __ホスト PC__

    ---

    ホスト PC 側ツール、フリート管理、crane やシミュレータとの連携。

    [:lucide-arrow-right: ホスト PC 側ツールへ](specs/pc/host_tools.md)

</div>

## その他のリポジトリ

<div class="grid cards" markdown>

-   :lucide-compass:{ .lg .middle .ibis-icon } __Crane ガイド__

    ---

    Vision から世界モデル、セッション、スキル、局所経路計画まで。Crane の動かし方と設計をまとめています。

    [:lucide-arrow-right: ガイドへ](others/crane/index.md)

-   :lucide-package:{ .lg .middle .ibis-icon } __Crane パッケージ一覧__

    ---

    Crane を構成する ROS 2 パッケージごとの役割と実装の解説です。

    [:lucide-arrow-right: パッケージへ](others/crane/packages/index.md)

-   :lucide-rocket:{ .lg .middle .ibis-icon } __Crane の環境構築・起動__

    ---

    開発環境のセットアップ、シミュレータでの起動、試合前のチェックリスト。

    [:lucide-arrow-right: セットアップへ](others/crane/setup.md)

-   :lucide-notebook-pen:{ .lg .middle .ibis-icon } __開発ログ__

    ---

    日々の開発で試したこと・分かったことの記録です。

    [:lucide-arrow-right: ログへ](others/crane/logs/2024-08-16.md)

-   :lucide-users:{ .lg .middle .ibis-icon } __他チーム調査__

    ---

    TIGERs、ZJUNLict、RoboDragons など強豪チームの公開資料の分析です。

    [:lucide-arrow-right: 調査へ](others/crane/other_team/tigers.md)

-   :lucide-library:{ .lg .middle .ibis-icon } __各リポジトリの文書__

    ---

    Main 基板・CM4・BLDC・Sub・昇圧基板のファームウェア、4 輪ステアリング機体、可視化ツールなどの文書です。

    [:lucide-arrow-right: 一覧へ](others/index.md)

-   :fontawesome-brands-github:{ .lg .middle .ibis-icon } __GitHub__

    ---

    ソースコードと仕様は ibis-ssl の GitHub で公開しています。

    [:lucide-arrow-right: ibis-ssl](https://github.com/ibis-ssl)

</div>
