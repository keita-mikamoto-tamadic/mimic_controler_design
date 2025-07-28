# 2次元倒立振子のラグランジュ未定乗数法を使わない地面拘束システム

## プロジェクト概要

本プロジェクトでは、2次元倒立振子（ボディ・ホイール系）において、ラグランジュ未定乗数法を使用せずに地面との拘束を実現するシステムを開発しました。

## システム構成

### 例として使用した物理モデル
- **ベースボディ**: 質量1kg、長さL=0.2m
- **ホイール**: 質量0.5kg、半径r=0.04m
- **4自由度**: `[x_base, z_base, φ_base, θ_wheel]`
- **拘束後**: 実質2自由度 `[x_base, φ_base]`

### 座標系定義
```
z ^
  |
  +----> x

ベース座標: (x_base, z_base)
ベース角度: φ_base (y軸周り回転)
ホイール角度: θ_wheel (y軸周り回転)
```

## 実装した拘束条件

### 1. ホイール中心高さ拘束
```python
# ホイール中心z座標 = r (地面からホイール半径の高さ)
wheel_center_z = z_base - L*cos(φ) = r
∴ z_base = r + L*cos(φ)
```

### 2. ホイール中心垂直速度拘束
```python
# d(wheel_center_z)/dt = 0 (ホイール中心は常に高さr)
d(wheel_center_z)/dt = dz_base + L*sin(φ)*dφ = 0
∴ dz_base = -L*sin(φ)*dφ
```

### 3. No-slip拘束（滑り無し条件）
```python
# ホイール中心の水平速度 = r × ホイール角速度
d(wheel_center_x)/dt = d(x_base + L*sin(φ))/dt = dx_base + L*cos(φ)*dφ = r*dθ
∴ dθ = (dx_base + L*cos(φ)*dφ) / r
```

## ファイル構成

```
src/test/simple_test/
├── simple_ip_fixed.py          # メインシミュレーション（最終完成版）
├── verify_noslip.py           # Rolling constraint検証
├── verify_true_noslip.py      # 真のno-slip拘束検証
├── debug_constraint.py        # 拘束計算デバッグ
└── simple_test.md            # 本説明文書
```

## 実行方法

### 基本シミュレーション
```bash
. bin/activate
python src/test/simple_test/simple_ip_fixed.py
```

### No-slip拘束検証
```bash
python src/test/simple_test/verify_true_noslip.py
```

## アニメーション表示内容

- **青線**: ベースボディ（長さL）
- **赤円**: ホイール（半径r）
- **赤点**: 接触点（常に地面上）
- **黒線**: 地面
- **リアルタイム情報**: 時間、座標、ホイール中心高さ、接触点高さ

## 理論的背景

### 2Dの運動方程式導出の具体的流れ

#### Step 0: 拘束条件の適用
```python
def solve_constraint_for_2dof_proper(q2, dq2, L, r, theta_prev=0.0, dt=0.01):
    x_base, phi_base = q2  # 独立変数（入力）
    dx_base, dphi_base = dq2
    
    # 従属変数を拘束条件から計算
    z_base = r + L * np.cos(phi_base)           # 位置拘束
    dz_base = -L * np.sin(phi_base) * dphi_base # 速度拘束
    dtheta_wheel = (dx_base + L * np.cos(phi_base) * dphi_base) / r  # no-slip拘束
    theta_wheel = theta_prev + dtheta_wheel * dt
    
    # 拘束を満たした完全な4自由度状態を構築
    q_full = np.array([x_base, z_base, phi_base, theta_wheel])
    dq_full = np.array([dx_base, dz_base, dphi_base, dtheta_wheel])
    
    return q_full, dq_full, theta_wheel
```

#### Step 1: Pinocchioで4自由度系の動力学項計算
```python
# Step0で拘束を満たした状態でPinocchio計算
q_full, dq_full = solve_constraint_for_2dof_proper(q2, dq2, L, r)

# Pinocchioで動力学項を一括計算（この時点でz_baseは既に決定済み）
pin.computeAllTerms(model, data, q_full, dq_full)
M_full = data.M        # 質量行列 (4×4)
C_full = data.C.dot(dq_full)  # コリオリ項 (4×1)
g_full = data.g        # 重力項 (4×1)
```

#### Step 2: 変換行列T構築（ヤコビアン行列）
```python
# T は ∂q_full/∂q2 のヤコビアン行列
T = np.zeros((4, 2))
T[0, 0] = 1.0                           # ∂x_base/∂x_base = 1
T[1, 1] = -L * np.sin(phi_base)         # ∂z_base/∂φ_base
T[2, 1] = 1.0                           # ∂φ_base/∂φ_base = 1  
T[3, 0] = 1.0/r                         # ∂θ_wheel/∂x_base
T[3, 1] = L * np.cos(phi_base) / r      # ∂θ_wheel/∂φ_base
```

#### Step 3: 2自由度系への縮約
```python
# 縮約された動力学項の計算
M_reduced = T.T @ M_full @ T    # 2×2質量行列
C_reduced = T.T @ C_full        # 2×1コリオリ項
g_reduced = T.T @ g_full        # 2×1重力項

# 2自由度での運動方程式
# M_reduced * ddq2 = tau2 - C_reduced - g_reduced
ddq2 = np.linalg.solve(M_reduced, tau2 - C_reduced - g_reduced)
```

### 使用するPinocchio関数
- `pin.computeAllTerms(model, data, q, dq)`: 質量行列、コリオリ、重力を一括計算
- `data.M`, `data.C`, `data.g`: 動力学項にアクセス
- `pin.forwardKinematics()`, `pin.updateFramePlacements()`: 運動学計算

### 物理的妥当性
本システムは以下の物理法則を満たします：
1. **ニュートンの運動法則**: 質量×加速度 = 力
2. **オイラーの回転方程式**: 慣性×角加速度 = トルク  
3. **拘束力の仮想仕事原理**: 拘束方向の仮想変位に対する仕事 = 0
4. **No-slip条件**: 接触点でのスリップ速度 = 0

## 参考文献・技術仕様

- **Pinocchio**: 剛体動力学ライブラリ
- **座標系**: 右手系（x右、z上）
- **角度**: ラジアン表記
- **数値積分**: オイラー法（dt=0.001～0.01）
- **描画**: matplotlib + animation

--