# crane_bringup

## 概要

Craneシステム全体の**起動統合パッケージ**として、複数のROS 2ノードやサービスを協調して起動し、システム全体の初期化とライフサイクル管理を行います。

## 主要機能

- **システム統合起動**: 全コンポーネントの協調起動
- **環境対応**: シミュレーション・実機環境の自動判定
- **パラメータ管理**: 環境に応じた設定の自動適用
- **診断統合**: システム全体の健全性監視

## 起動スクリプト

### crane.launch.xml

```bash
# メインシステム起動（シミュレーション）
ros2 launch crane_bringup crane.launch.xml sim:=true

# 実機システム起動
ros2 launch crane_bringup crane.launch.xml sim:=false
```

### data.launch.py

```bash
# データ処理パイプライン起動
ros2 launch crane_bringup data.launch.py
```

## アーキテクチャ上の役割

Craneシステムの**統合・起動管理層**として、複雑なマルチノードシステムの起動順序制御と依存関係管理を担います。

## 最近の開発状況

🟢 **安定**: 起動システムとして成熟しており、新コンポーネント追加時の統合や起動手順の最適化が行われています。

---

**関連パッケージ**: [crane_description](./crane_description.md) | 全システムパッケージ
