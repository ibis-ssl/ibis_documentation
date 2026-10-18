---
title: ホーム
hide:
  - navigation
  - toc
---

<div class="ibis-hero" markdown>

<span class="ibis-hero__eyebrow">RoboCup Soccer Small Size League</span>

# ibis 公開ドキュメント

RoboCup SSL に出場するチーム ibis が、Orion ロボットの仕様と、AI・ファームウェア・ツールの開発で得た知見を公開しています。

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

ibis-ssl の各リポジトリにある文書です。元のリポジトリから毎日取得しています。[一覧](others/index.md)

<div class="grid cards" markdown>

-   :lucide-compass:{ .lg .middle .ibis-icon } __crane__

    ---

    AI「Crane」のガイド、ROS 2 パッケージの解説、環境構築、開発ログ、他チーム調査。

    [:lucide-arrow-right: crane](others/crane/index.md)

-   :lucide-circuit-board:{ .lg .middle .ibis-icon } __G474_Orion_main__

    ---

    Orion の Main 基板（STM32G474）ファームウェア。

    [:lucide-arrow-right: G474_Orion_main](others/G474_Orion_main/overview.md)

-   :lucide-cpu:{ .lg .middle .ibis-icon } __Orion_CM4__

    ---

    Orion の CM4 上のソフトウェアとホスト PC 側ツール。

    [:lucide-arrow-right: Orion_CM4](others/Orion_CM4/overview.md)

-   :lucide-zap:{ .lg .middle .ibis-icon } __Orion の各基板__

    ---

    BLDC 基板、Sub 基板、昇圧・キッカー基板（STM32F303）のファームウェア。

    [:lucide-arrow-right: Orion_F303_BLDC](others/Orion_F303_BLDC/overview.md)

    [:lucide-arrow-right: Orion_F303_sub](others/Orion_F303_sub/overview.md)

    [:lucide-arrow-right: F303_boost](others/F303_boost/overview.md)

-   :lucide-car:{ .lg .middle .ibis-icon } __4 輪ステアリング機体__

    ---

    メイン制御ボードとステアリング制御基板のファームウェア。

    [:lucide-arrow-right: 4WS_MainFW](others/4WS_MainFW/overview.md)

    [:lucide-arrow-right: SwerveControllerF303](others/SwerveControllerF303/system.md)

-   :lucide-chart-scatter:{ .lg .middle .ibis-icon } __foxglove_crane_visualizer__

    ---

    Crane 用の Foxglove Studio 可視化拡張。

    [:lucide-arrow-right: foxglove_crane_visualizer](others/foxglove_crane_visualizer/README.md)

</div>

ソースコードは [ibis-ssl の GitHub](https://github.com/ibis-ssl) で公開しています。
