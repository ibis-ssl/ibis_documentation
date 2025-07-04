# closest_point_vendor

## 概要

**最近点計算ライブラリ**のベンダーパッケージです。幾何学的最近点計算アルゴリズムを提供し、ロボット経路計画・衝突判定・距離計算等で使用される基礎的な幾何学演算を支援します。

## 主要機能

- **点と線分の最近点**: 2D/3D空間での高速計算
- **点と多角形の最近点**: 複雑な形状への対応
- **距離計算**: 各種幾何プリミティブ間の距離
- **高精度計算**: 数値誤差を最小化した実装

## 計算機能

- **点-点距離**: 基本的なユークリッド距離
- **点-線分距離**: 線分上の最近点
- **点-円距離**: 円周上の最近点
- **点-多角形距離**: 複雑形状の最近点

## アーキテクチャ上の役割

Craneシステムの**幾何学計算基盤**として、crane_geometry・crane_local_planner等で使用される基礎計算ライブラリです。

## 使用シーン

- **経路計画**: 障害物との距離計算
- **衝突判定**: 最近接距離による衝突検出
- **位置制御**: 目標位置への最短経路
- **フィールド境界**: フィールド端への距離

## パフォーマンス

- **計算速度**: μ秒オーダーの高速計算
- **精度**: 機械精度レベル
- **メモリ効率**: 最小限のメモリ使用

## 最近の開発状況

🟢 **安定**: 基礎計算ライブラリとして成熟しており、計算精度の向上と新しい幾何プリミティブへの対応が継続的に行われています。

---

**関連パッケージ**: [crane_geometry](./crane_geometry.md) | [crane_local_planner](./crane_local_planner.md)
