# crane_visualization_aggregator

## 概要

Craneシステムの**可視化データ統合**を担うパッケージです。複数のデータソースからの可視化情報を集約し、統一的な可視化インターフェースとして外部可視化システムに提供します。

## 主要機能

- **データ統合**: 複数ノードからの可視化データの集約
- **統一インターフェース**: 一元化された可視化データ配信
- **リアルタイム処理**: 低遅延での可視化データ更新
- **外部ツール連携**: RViz、Foxglove等との統合

## アーキテクチャ上の役割

Craneシステムの**可視化統合基盤**として、分散された可視化情報を統合し、デバッグ・分析・観戦用の統一的な可視化環境を提供します。

## 統合データ

- **ロボット状態**: 位置・姿勢・役割・状態
- **ボール情報**: 軌道・予測・物理状態
- **戦術情報**: フォーメーション・戦略・判定
- **システム診断**: エラー・警告・パフォーマンス

## 使用方法

```bash
# 可視化統合システム起動
ros2 run crane_visualization_aggregator visualization_aggregator_node
```

## 最近の開発状況

🟢 **安定**: 可視化統合システムとして基本機能が確立しており、新しい可視化データ型の追加や統合効率の向上が継続的に行われています。

---

**関連パッケージ**: [crane_visualization_interfaces](./crane_visualization_interfaces.md) | [consai_visualizer](./consai_visualizer.md)
