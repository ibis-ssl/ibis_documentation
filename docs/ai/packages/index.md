# Craneパッケージドキュメント ポータル

> **📋 ドキュメント保守ハブ**  
> このページは各パッケージドキュメントの最新状況を集約し、アップデート対象の洗い出しと進捗共有を簡潔に把握できるようにするためのポータルです。

## プロジェクト概要

**Crane** は ibis-ssl チームが開発するRoboCup Small Size League (SSL) 用の自律ロボットサッカーシステムです。ROS 2 Jazzy ベースで構築された、小型自律ロボットチームによるサッカー試合を制御するAIフレームワークです。

### システム特徴

- **リアルタイム制御**: SSL競技規定に準拠した高精度な制御系
- **マルチロボット協調**: RVO2アルゴリズムによる衝突回避
- **プラグインアーキテクチャ**: 戦略・スキルの拡張性
- **3D物理モデル**: ボール軌道の高精度予測

### 最近の開発活動

- **2025年6月**: JapanOpen2025での実戦運用完了・全32パッケージの整備確認
- **2025年前半期**: システム統合・安定性向上・ルール違反対策の仕上げ
- **開発状況**: 安定運用フェーズ（大会検証済み）

---

## 📦 パッケージ一覧・作業状況

### 🔥 Core系パッケージ群 (14/14) - 高優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_msgs](./crane_msgs.md) | メッセージ定義基盤 | 🟢 安定 | ✅ 作成済み | Session 1 |
| [crane_world_model_publisher](./crane_world_model_publisher.md) | 世界状態推定・トラッキング | 🔴 高活動 | ✅ 作成済み | Session 1 |
| [crane_robot_skills](./crane_robot_skills.md) | ロボットスキルライブラリ | 🟡 中活動 | ✅ 作成済み | Session 1 |
| [crane_local_planner](./crane_local_planner.md) | 経路計画・衝突回避 | 🔴 高活動 | ✅ 作成済み | 完了 |
| [crane_game_analyzer](./crane_game_analyzer.md) | 試合状況分析 | 🟡 中活動 | ✅ 作成済み | 完了 |
| [crane_play_switcher](./crane_play_switcher.md) | プレイ自動選択 | 🟡 中活動 | ✅ 作成済み | 完了 |
| [crane_sender](./crane_sender.md) | ロボットコマンド送信 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_bringup](./crane_bringup.md) | システム起動統合 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_description](./crane_description.md) | パラメータ管理 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_robot_receiver](./crane_robot_receiver.md) | ロボット状態受信 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_visualization_interfaces](./crane_visualization_interfaces.md) | 可視化インターフェース | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_gui](./crane_gui.md) | GUI（開発中止） | ⚫ 無効 | ✅ 作成済み | 完了 |
| [crane_simple_ai](./crane_simple_ai.md) | 簡易AI制御 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_speaker](./crane_speaker.md) | 音声出力システム | 🟢 安定 | ✅ 作成済み | 完了 |

### 🎯 Session系パッケージ群 (2/2) - 高優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_session_controller](./crane_session_controller.md) | 試合統括・ゲーム状態管理 | 🟡 中活動 | ✅ 作成済み | Session 1 |
| [crane_planner_plugins](./crane_planner_plugins.md) | 戦略プランナープラグイン | 🔴 高活動 | ✅ 作成済み | 完了 |

### 🔧 Utility系パッケージ群 (10/10) - 中優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_geometry](./crane_geometry.md) | 幾何学計算ライブラリ | 🔴 高活動 | ✅ 作成済み | 完了 |
| [crane_physics](./crane_physics.md) | 物理計算・ボールモデル | 🔴 高活動 | ✅ 作成済み | 完了 |
| [crane_comm](./crane_comm.md) | 通信ユーティリティ | 🟡 中活動 | ✅ 作成済み | 完了 |
| [crane_msg_wrappers](./crane_msg_wrappers.md) | メッセージラッパー | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_clock_publisher](./crane_clock_publisher.md) | システム時刻同期 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_grsim_operator](./crane_grsim_operator.md) | grSim操作 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_lint_common](./crane_lint_common.md) | 共通リント設定 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_teleop](./crane_teleop.md) | 遠隔操作 | 🟢 安定 | ✅ 作成済み | 完了 |
| [crane_visualization_aggregator](./crane_visualization_aggregator.md) | 可視化データ統合 | 🟢 安定 | ✅ 作成済み | 完了 |
| ~~[crane_basics](./crane_basics.md)~~ | ~~基礎ユーティリティライブラリ~~ | ⚫ 解体済 | ⚠️ 非推奨 | - |

### 📡 SSL通信系パッケージ群 (3/3) - 中優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [robocup_ssl_comm](./robocup_ssl_comm.md) | SSL通信プロトコル処理 | 🟢 安定 | ✅ 作成済み | 完了 |
| [robocup_ssl_msgs](./robocup_ssl_msgs.md) | SSL公式メッセージ定義 | 🟢 安定 | ✅ 作成済み | 完了 |
| [consai_visualizer](./consai_visualizer.md) | SSL可視化GUI | 🟢 安定 | ✅ 作成済み | 完了 |

### 📚 3rdparty系パッケージ群 (3/3) - 低優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [rvo2_vendor](./rvo2_vendor.md) | RVO2衝突回避アルゴリズム | 🟢 安定 | ✅ 作成済み | 完了 |
| [matplotlib_cpp_17_vendor](./matplotlib_cpp_17_vendor.md) | C++17対応matplotlib | 🟢 安定 | ✅ 作成済み | 完了 |
| [closest_point_vendor](./closest_point_vendor.md) | 最近点計算ライブラリ | 🟢 安定 | ✅ 作成済み | 完了 |

---

## 🛠️ ドキュメント更新手順

### 推奨ワークフロー

1. 対象パッケージを表から確認し、担当欄を更新
2. ドキュメントを見直し、実装と差分がある箇所のみを書き換え
3. 更新後は「ドキュメント状況」と「担当セッション」を反映
4. 参照リンクと隣接パッケージの記述が矛盾しないか確認

### テンプレート（参考）

```markdown
# パッケージ名

## 概要
[パッケージの目的と概要]

## 主要機能
- 機能1
- 機能2

## アーキテクチャ上の役割
[システム内での位置づけ]

## コンポーネント
### ノード
- ノード名: 機能説明

### ライブラリ
- ライブラリ名: 機能説明

## 依存関係
- パッケージ依存
- システム依存

## 使用方法
[基本的な使用例]

## 最近の開発状況
[2025年JapanOpen後の開発状況と主要な変更]
```

---

## 📊 進捗サマリー

- **全体進捗**: 32/32 パッケージ (100%) ✅ **完了**
- **Core系**: 14/14 完了 (100%) ✅
- **Session系**: 2/2 完了 (100%) ✅
- **Utility系**: 10/10 完了 (100%) ✅
- **SSL通信系**: 3/3 完了 (100%) ✅
- **3rdparty系**: 3/3 完了 (100%) ✅

**最終更新**: 2025-06-22

---

## 🔗 関連リンク

- [メインドキュメント](../index.md)
- [アーキテクチャ概要](../README.md)
- [開発ログ](../logs/)
- [Claude Code開発ドキュメント](../CLAUDE.md)

---

## 📝 注意事項

1. **並列作業時**: 担当セッション欄を必ず更新してください
2. **ドキュメント品質**: 技術的正確性と日本語統一性を保持
3. **リンク整合性**: パッケージ間参照の正確性を確認
4. **定期更新**: 開発進捗に応じてドキュメント更新
