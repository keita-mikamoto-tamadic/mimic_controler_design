# URDF Full Robot 3D 実装プラン

## 概要
**mimic_v1.urdf**モデルを使用した3次元フルロボット拘束付き動力学シミュレーション。single_legプロジェクトで確立した高精度ノンスリップ拘束手法を、左右両脚を持つフルロボットモデルに拡張する。

## 使用モデル
- **URDFファイル**: `urdf/mimic_v1.urdf`
- **ロボット構成**: 二足歩行ロボット（6関節）
  - base_link
  - 左脚: upper_link_L → lower_link_L → wheel_L
  - 右脚: upper_link_R → lower_link_R → wheel_R

## モデル仕様

### 3次元空間での自由度構成
- **フローティングベース**: 6自由度
  - x, y, z方向並進
  - roll, pitch, yaw回転
- **関節自由度**: 6自由度
  - 左脚: upper_L, lower_L, wheel_L（各1自由度）
  - 右脚: upper_R, lower_R, wheel_R（各1自由度）
- **総自由度**: 12自由度

### 拘束条件
1. **左ホイール接地拘束**: wheel_L底面が地面（z=0）に接触
2. **右ホイール接地拘束**: wheel_R底面が地面（z=0）に接触
3. **左ホイールノンスリップ拘束**: 左ホイール中心移動距離 = 左タイヤ回転距離
4. **右ホイールノンスリップ拘束**: 右ホイール中心移動距離 = 右タイヤ回転距離
- **拘束後自由度**: 12 - 4 = 8自由度

### 独立変数の選択
```python
# 8独立変数
[x_base, y_base, yaw,  # ベース水平位置と回転
 phi_L_upper, phi_L_lower,  # 左脚関節角度
 phi_R_upper, phi_R_lower,  # 右脚関節角度
 phi_extra]  # 追加自由度（roll or pitch）
```

### 従属変数（拘束から決定）
```python
# 4従属変数
[z_base,  # ベース高度（接地拘束から）
 roll or pitch,  # ベース姿勢（接地拘束から）
 theta_wheel_L,  # 左タイヤ角度（ノンスリップから）
 theta_wheel_R]  # 右タイヤ角度（ノンスリップから）
```

## 実装方針

### 1. Single_leg手法の継承と拡張
- **位置ベース拘束**: 両ホイール中心位置から直接タイヤ角度を計算
- **累積角度追跡**: 左右独立の2π跨ぎ問題解決
- **高精度実現**: 機械精度レベル（10^-16）の拘束満足

### 2. 3次元特有の課題対応
- **姿勢決定**: 2つの接地点から適切なroll/pitchを計算
- **対称性考慮**: 左右脚の幾何学的対称性を活用
- **数値安定化**: 4拘束同時満足のための反復解法

### 3. システムアーキテクチャ
```
dynamics_3d.py          # 純粋な動力学計算
├── load_model()        # URDFモデル読み込み
├── compute_constrained_config()  # 拘束満足構成計算
└── simulate_dynamics() # 運動方程式積分

constraints_3d.py       # 拘束条件処理
├── compute_wheel_angles()  # ノンスリップからタイヤ角度
├── compute_base_pose()     # 接地拘束からベース姿勢
└── verify_constraints()    # 拘束精度検証

animation_3d.py         # 3D可視化
├── create_3d_animation()   # 3次元骨格アニメーション
└── plot_trajectories()     # 軌跡・角度変化表示

verification_3d.py      # 独立検証システム
├── verify_ground_contact() # 接地拘束検証
└── verify_noslip()        # ノンスリップ拘束検証
```

## 実装ステップ

### Phase 1: 基礎構築
1. URDFモデル読み込みと構造解析
2. 初期姿勢計算（4拘束を満たす構成）
3. 基本的な順運動学検証

### Phase 2: 拘束システム実装
4. 接地拘束によるベース位置・姿勢決定
5. ノンスリップ拘束による左右タイヤ角度計算
6. 拘束ヤコビアンの導出と検証

### Phase 3: 動力学計算
7. 8自由度縮約運動方程式の構築
8. 数値積分（Runge-Kutta法）
9. 拘束力の計算と物理妥当性確認

### Phase 4: 可視化と検証
10. 3Dアニメーション実装
11. 拘束精度の定量評価
12. エネルギー保存則の確認

## 技術的詳細

### Pinocchio API活用
```python
# モデル構築
model = pin.buildModelFromUrdf("urdf/mimic_v1.urdf", pin.JointModelFreeFlyer())

# 拘束処理
pin.forwardKinematics(model, data, q)
J_wheel_L = pin.getFrameJacobian(model, data, wheel_L_frame, pin.ReferenceFrame.WORLD)
J_wheel_R = pin.getFrameJacobian(model, data, wheel_R_frame, pin.ReferenceFrame.WORLD)

# 動力学
pin.crba(model, data, q)  # 慣性行列
pin.computeGeneralizedGravity(model, data, q)  # 重力項
```

### 数値安定化手法
- **拘束違反の最小化**: Newton-Raphson法による反復修正
- **特異点回避**: 適切な初期姿勢選択
- **エラー累積防止**: 定期的な拘束再投影

## 期待される成果

### 物理現象
- **3次元振り子運動**: 左右脚の連成による複雑な動作
- **姿勢変化**: roll/pitch/yawの自然な変動
- **対称性の破れ**: 初期条件による非対称運動

### 技術的成果
- **Single_leg手法の完全スケーリング**: 1脚→2脚、2D→3D
- **高精度4拘束同時満足**: 機械精度レベルの実現
- **実用的3Dシミュレーション**: リアルタイム可視化

## 参考プロジェクト

### single_legプロジェクト（直接継承）
- **場所**: `src/test/urdf_single_leg/`
- **継承技術**:
  - 位置ベースノンスリップ拘束
  - 累積角度追跡（2π跨ぎ対策）
  - モジュール分離設計
  - 独立検証システム

### full_robot_dynamicsプロジェクト（参考）
- **場所**: `src/test/full_robot_dynamics/`
- **参考点**:
  - mimic_v1.urdfの構造理解
  - 2脚接地拘束の基本実装
  - 関節インデックスマッピング

## 成功指標
- ✅ 接地拘束誤差 < 10^-14
- ✅ ノンスリップ拘束誤差 < 10^-14
- ✅ 5秒以上の安定シミュレーション
- ✅ エネルギー保存誤差 < 1%
- ✅ リアルタイム3Dアニメーション