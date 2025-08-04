# 拘束適用単脚2D線形化システム実装プラン

## プロジェクト概要
既存の`urdf_single_leg`の拘束実装を活用し、**真の4DOFシステム**として線形化を行います。`01_ApproximationModel.md`の理論に完全準拠した、接地・no-slip・幾何拘束を適用した状態空間モデルを構築します。

## 技術の流用
- urdf_single_leg内のファイルはコピーして使用可能。
- なるべくライブラリや、既存実装の組み合わせで行うこと。自前の実装は避ける。

## 技術背景

### 既存実装の活用
- **ベースシステム**: `src/test/urdf_single_leg/noslip_constrained.py`
- **実績ある拘束処理**: 接地拘束 + no-slip拘束で6DOF→4DOF実現済み
- **動力学関数**: `compute_4dof_noslip_dynamics()` による確実な拘束適用

### 理論的根拠 (01_ApproximationModel.md準拠)
- **幾何拘束**: `phi2 = -2 * phi1` (膝関節は股関節の-2倍)
- **平衡点**: `phi1 = 52°, phi2 = -104°`
- **トルク拘束**: `tau2 = f(phi1) * tau1, f(52°) ≈ -8.228`

## システム設計

### 1. 拘束システムの統合
**物理拘束** (既存実装活用):
- **接地拘束**: ホイール底面をz=0に固定 → z_base従属
- **no-slip拘束**: ホイール回転とベース移動の関係 → phi3従属
- **2D拘束**: y,roll,yaw固定 → 3DOF削減

**重要な前提: 初期ベース高さの算出**
```
base_height = WHEEL_RADIUS - wheel_pos[2]
```
この式により、任意の関節角度(phi1, phi2)に対して：
1. **仮構成設定**: x_base=0でneutral構成を作成
2. **順運動学実行**: pin.forwardKinematics()でホイール位置を計算
3. **接地拘束適用**: ホイール底面がz=0となるベース高度を逆算

**結果**: 9DOF → **4DOF物理系** (ベース高度は関節角度の従属変数)

### 2. 状態空間定義
**4DOF状態ベクトル** (物理拘束適用後):
```
z_phys = [x_base, phi1, phi2, dx_base, dphi1, dphi2]ᵀ  (6次元)
```

**従属変数** (拘束により自動決定):
- `z_base`: 接地拘束 → `compute_base_position_from_constraint(phi1, phi2)`
  - 実装: `base_height = WHEEL_RADIUS - wheel_pos[2]`
  - ホイール位置を順運動学で計算し、底面がz=0となる高度を逆算
- `phi3`: no-slip拘束 → `compute_wheel_angle_from_position(x_base, phi1, phi2)`
  - 実装: `theta_wheel = -wheel_center[0] / WHEEL_RADIUS`
- `dz_base`: 接地点速度=0 → ヤコビアン拘束から計算
- `dphi3`: no-slip条件 → `compute_wheel_angular_velocity_from_center_velocity()`

### 3. 幾何拘束による最終縮約
**01_ApproximationModel.md適用**:
```
phi2 = -2 * phi1  (構造的拘束)
dphi2 = -2 * dphi1
```

**最終縮約状態**:
```
y = [x_base, pitch_base, phi1(hip_pitch), dx_base, dpitch_base ,dphi1]ᵀ  (6次元)
```

### 4. 入力系統
**トルク拘束** (01_ApproximationModel.md):
```
下記のように線形で近似すること。
tau2 = f(phi1) * tau1
f(phi1) = -0.00214 * phi1² + 0.2636 * phi1 - 8.4662
f(52°) ≈ -8.228  (線形近似)
```

**独立入力**:
```
u = [tau1, tau3]ᵀ  (2次元)
```

## 実装アーキテクチャ

### ファイル構成
```
src/test/single_leg_linearization/
├── single_leg_linearization.md        # 本文書

```

### クラス設計

#### 1. ConstrainedDynamics
**目的**: 既存拘束実装の統合クラス化
**機能**:
- `noslip_constrained.py`の関数群をオブジェクト指向化
- `compute_base_position_from_constraint()`: 順運動学による接地拘束適用
- `compute_wheel_angle_from_position()`: no-slip拘束によるホイール角度計算
- 4DOF物理拘束システムの状態方程式提供
- 拘束整合性の検証機能

#### 2. EquilibriumSolver  
**目的**: 拘束下での平衡点計算
**機能**:
- 接地・重力・幾何拘束の同時満足解
- この機能で言う平衡点は、ベースのz位置のこと。（ホイールが）
「重要な前提: 初期ベース高さの算出」と同じ意味。

#### 3. ConstrainedLinearization
**目的**: 拘束状態空間での線形化
**機能**:
- 自前の実装は行わず、scipyを使用する。
- 6次元状態空間(z_phys)でのテイラー展開
- 数値微分による∂f/∂x, ∂f/∂u計算
- 拘束整合性を保った線形化

#### 4. ConstraintReduction
**目的**: 幾何拘束による最終縮約
**機能**:
- phi2=-2*phi1拘束の適用
- 6DOF→4DOF状態空間変換
- トルク拘束による入力空間縮約

#### 5. LinearizedControllerBase
**目的**: A,B行列と制御設計基盤
**機能**:
- 最終4×4 A行列, 4×2 B行列の提供
- 可制御性・安定性解析
- LQR制御器設計インターフェース

## 期待される成果

### 数学的モデル
**状態方程式**:
```
dy/dt = A * y + B * u
```
ここで:
- `y = [x_base, phi1, dx_base, dphi1]ᵀ` (4次元)
- `u = [tau1, tau3]ᵀ` (2次元)
- `A`: 4×4行列 (フルランク期待)
- `B`: 4×2行列 (フルランク期待)

### システム特性
- **完全可制御**: rank([B, AB, A²B, A³B]) = 4
- **適切な安定性**: 平衡点周りで妥当な固有値
- **物理整合性**: 全拘束条件の同時満足

### A行列の期待構造 (4×4)
```
A = [  0   0   1   0 ]  # dx_base  = dx_base
    [  0   0   0   1 ]  # dphi1   = dphi1
    [a31 a32 a33 a34]  # ddx_base = f(重力,慣性,拘束)
    [a41 a42 a43 a44]  # ddphi1  = f(重力,慣性,拘束)
```

### B行列の期待構造 (4×2)
```
B = [ 0   0 ]  # dx_base  ← [tau1, tau3]
    [ 0   0 ]  # dphi1   ← [tau1, tau3]  
    [b31 b32]  # ddx_base ← [tau1, tau3]
    [b41 b42]  # ddphi1  ← [tau1, tau3]
```

## 実装順序

### Phase 1: 基盤システム (High Priority)
1. **ConstrainedDynamics**: 既存関数の統合とクラス化
2. **EquilibriumSolver**: 拘束下平衡点の数値計算
3. **基本検証**: 拘束適用と平衡点の妥当性確認

### Phase 2: 線形化システム (Medium Priority)  
4. **ConstrainedLinearization**: 6DOF状態空間での線形化
5. **ConstraintReduction**: 幾何拘束による4DOF縮約
6. **行列検証**: A,B行列の数値的・理論的妥当性

### Phase 3: 制御基盤 (Medium Priority)
7. **LinearizedControllerBase**: 最終システム行列と解析
8. **統合システム**: 全プロセスの統合実行
9. **性能検証**: 可制御性・応答特性の確認

## 成功基準

### 技術的目標
- ✅ **4×4フルランクA行列**: 適切な動力学表現
- ✅ **4×2フルランクB行列**: 完全可制御性
- ✅ **拘束整合性**: 全拘束条件の数値的満足
- ✅ **平衡点妥当性**: 物理的に意味のある平衡状態

### 理論的整合性
- ✅ **01_ApproximationModel.md準拠**: 全仕様の完全実装
- ✅ **既存実装継承**: `urdf_single_leg`の確実な拘束処理活用
- ✅ **制御設計可能**: LQR等の古典制御手法適用可能

このシステムにより、単脚ロボットの真の動力学に基づく制御器設計が可能となり、二脚ロボット制御の基礎技術確立に貢献します。