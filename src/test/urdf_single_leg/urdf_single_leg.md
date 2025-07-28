# URDF Single Leg 実装プラン

## 概要
mimic_v1_single_leg.urdfモデルを使用した拘束付き動力学シミュレーション

## モデル構成
- **自由度**: 3自由度（全て回転関節）
  - upper_link_R_joint: base_linkとupper_link_Rを接続
  - lower_link_R_joint: upper_link_Rとlower_link_Rを接続
  - wheel_R_joint: lower_link_Rとwheel_Rを接続
- **リンク構造**: base_link → upper_link_R → lower_link_R → wheel_R

## 実装方針

### 1. システムのモデリング
- URDFからPinocchioモデルを構築
- 3関節のロボットとしてモデル化
- タイヤ（wheel_R）の接地点を拘束として扱う

### 2. 拘束条件
- タイヤは地面から絶対に浮かない（z座標が常に一定）
- この拘束により自由度が3から減少する
- simple_testと同様の拘束単純化アプローチを採用

### 3. 動力学計算
- Pinocchioを使用した運動方程式の導出
- 拘束による縮約を実施
- 重力のみの自由応答をシミュレート（制御なし）

### 4. 期待される動作
- ボディ（base_link）が重力で落下
- タイヤが地面に固定されているため、2重振り子のような挙動
- 3関節による複雑な振動パターン

## 実装ステップ
1. URDFモデルの読み込みと初期可視化
2. 拘束条件の数式化
3. 初期姿勢の計算（タイヤ接地を満たす姿勢）
4. 運動方程式の導出（Pinocchio使用）
5. 拘束による縮約（自由度削減）
6. 数値積分によるシミュレーション
7. 2Dアニメーション作成

## 技術的詳細
- Pinocchio APIの活用
- 拘束はヤコビアンを使用して表現
- 数値積分にはRunge-Kutta法を使用予定